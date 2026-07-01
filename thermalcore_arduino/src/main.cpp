/*
 * ThermalCore Optimizer - HMI Firmware (Modulo 1)
 * ------------------------------------------------
 * Hardware: Arduino Uno + LCD Keypad Shield (D1 Robot / DFRobot).
 *   - LCD 16x2 em modo 4-bit (RS=8, E=9, D4..D7=4..7).
 *   - Botoes UP/DOWN/LEFT/RIGHT/SELECT em divisor de tensao no pino A0.
 *
 * Papel da HMI: coletar os requisitos de projeto (q_c e T_max), enviar ao
 * motor de otimizacao (Python) via Serial em JSON e exibir a geometria otima
 * retornada. O firmware nao faz calculo termico; ele e a interface.
 *
 * Protocolo Serial (9600 baud, linhas terminadas em '\n'):
 *   TX (request):  {"q_c":20.00,"T_max":85.00}
 *   RX (response): {"N":14,"t":1.20}
 */

#include <Arduino.h>
#include <LiquidCrystal.h>

// --------------------------------------------------------------------------
// Mapeamento de hardware (LCD Keypad Shield)
// --------------------------------------------------------------------------
static LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
static const uint8_t PIN_KEYPAD    = A0;
static const uint8_t PIN_BACKLIGHT = 10;

static const unsigned long SERIAL_BAUD    = 9600UL;
static const unsigned long REPLY_TIMEOUT  = 8000UL;   // ms aguardando resposta

// --------------------------------------------------------------------------
// Teclado analogico (divisor de tensao no A0)
// --------------------------------------------------------------------------
enum Button : uint8_t {
  BTN_NONE = 0,
  BTN_RIGHT,
  BTN_UP,
  BTN_DOWN,
  BTN_LEFT,
  BTN_SELECT
};

// Le o botao "cru" a partir do ADC. Limiares tipicos do shield DFRobot.
static Button readRawButton() {
  int adc = analogRead(PIN_KEYPAD);
  if (adc > 1000) return BTN_NONE;
  if (adc < 50)   return BTN_RIGHT;
  if (adc < 195)  return BTN_UP;
  if (adc < 380)  return BTN_DOWN;
  if (adc < 555)  return BTN_LEFT;
  if (adc < 790)  return BTN_SELECT;
  return BTN_NONE;
}

// Debounce + deteccao de borda + auto-repeticao para UP/DOWN mantidos.
// Retorna um evento de botao (borda de descida logica) ou BTN_NONE.
static Button pollButton() {
  static const unsigned long DEBOUNCE_MS = 30;
  static const unsigned long REPEAT_DELAY = 400;   // 1a repeticao apos segurar
  static const unsigned long REPEAT_RATE  = 150;   // repeticoes subsequentes

  static Button stable   = BTN_NONE;      // botao debounced atual
  static Button lastRaw  = BTN_NONE;
  static unsigned long lastChange = 0;
  static unsigned long nextRepeat = 0;

  unsigned long now = millis();
  Button raw = readRawButton();

  if (raw != lastRaw) {          // sinal mudou: reinicia janela de debounce
    lastRaw = raw;
    lastChange = now;
    return BTN_NONE;
  }
  if (now - lastChange < DEBOUNCE_MS) {
    return BTN_NONE;             // ainda instavel
  }

  if (raw != stable) {           // novo estado estavel confirmado
    stable = raw;
    if (stable != BTN_NONE) {
      nextRepeat = now + REPEAT_DELAY;
      return stable;             // dispara no instante da pressao
    }
    return BTN_NONE;
  }

  // Botao mantido: auto-repeticao apenas para UP/DOWN (ajuste de valor).
  if (stable == BTN_UP || stable == BTN_DOWN) {
    if (now >= nextRepeat) {
      nextRepeat = now + REPEAT_RATE;
      return stable;
    }
  }
  return BTN_NONE;
}

// --------------------------------------------------------------------------
// Modelo da HMI
// --------------------------------------------------------------------------
struct DesignInput {
  float q_c;    // dissipacao termica do chip [W]
  float t_max;  // temperatura critica do chip [C]
};

struct DesignResult {
  int   n;      // numero de aletas
  float t;      // espessura da aleta [mm]
};

enum Field : uint8_t { FIELD_QC = 0, FIELD_TMAX = 1 };

enum UiState : uint8_t {
  ST_INPUT,     // editando requisitos
  ST_WAIT,      // request enviado, aguardando resposta
  ST_RESULT,    // exibindo geometria otima
  ST_TIMEOUT    // nenhuma resposta a tempo
};

static DesignInput  input   = { 20.0f, 85.0f };
static DesignResult result  = { 0, 0.0f };
static Field        field   = FIELD_QC;
static UiState      state   = ST_INPUT;
static unsigned long waitStart = 0;

// Limites e passos de edicao
static const float QC_MIN = 0.0f,   QC_MAX = 200.0f, QC_STEP = 1.0f;
static const float TM_MIN = 20.0f,  TM_MAX = 150.0f, TM_STEP = 1.0f;

static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// --------------------------------------------------------------------------
// Recepcao Serial (linha JSON de resposta)
// --------------------------------------------------------------------------
static char    rxBuf[48];
static uint8_t rxLen = 0;

// Extrai o valor numerico da chave "key" dentro de uma linha JSON simples.
// Retorna true e preenche *out em caso de sucesso.
static bool jsonNumber(const char* json, const char* key, float* out) {
  const char* p = strstr(json, key);
  if (!p) return false;
  p += strlen(key);
  while (*p == ' ' || *p == ':' || *p == '"') p++;  // pula ": e espacos
  char* end = nullptr;
  float v = strtod(p, &end);
  if (end == p) return false;
  *out = v;
  return true;
}

// Faz parsing de {"N":14,"t":1.20}. Retorna true se ambos campos existem.
static bool parseResult(const char* json, DesignResult* r) {
  float n, t;
  if (!jsonNumber(json, "\"N\"", &n)) return false;
  if (!jsonNumber(json, "\"t\"", &t)) return false;
  r->n = (int)(n + 0.5f);
  r->t = t;
  return true;
}

// Acumula caracteres da Serial ate '\n' e tenta interpretar a resposta.
static void serviceSerialRx() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      rxBuf[rxLen] = '\0';
      if (rxLen > 0 && parseResult(rxBuf, &result)) {
        state = ST_RESULT;
      }
      rxLen = 0;
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = c;
    } else {
      rxLen = 0;  // linha longa demais: descarta
    }
  }
}

static void sendRequest() {
  Serial.print(F("{\"q_c\":"));
  Serial.print(input.q_c, 2);
  Serial.print(F(",\"T_max\":"));
  Serial.print(input.t_max, 2);
  Serial.println(F("}"));
}

// --------------------------------------------------------------------------
// Renderizacao do LCD
// --------------------------------------------------------------------------
// Imprime "label val unit" preenchendo 16 colunas; marca o campo ativo com '>'.
static void printField(uint8_t row, bool active, const char* label,
                       float val, const char* unit) {
  lcd.setCursor(0, row);
  lcd.print(active ? '>' : ' ');
  lcd.print(label);
  lcd.print(val, 1);
  lcd.print(' ');
  lcd.print(unit);
  lcd.print(F("    "));  // limpa residuos da linha
}

static void renderInput() {
  printField(0, field == FIELD_QC,   "q_c  ", input.q_c,   "W");
  printField(1, field == FIELD_TMAX, "Tmax ", input.t_max, "C");
}

static void renderWait() {
  lcd.setCursor(0, 0);
  lcd.print(F("Otimizando...   "));
  lcd.setCursor(0, 1);
  lcd.print(F("q="));
  lcd.print(input.q_c, 0);
  lcd.print(F("W Tmax="));
  lcd.print(input.t_max, 0);
  lcd.print(F("C  "));
}

static void renderResult() {
  lcd.setCursor(0, 0);
  lcd.print(F("Geometria otima "));
  lcd.setCursor(0, 1);
  lcd.print(F("N="));
  lcd.print(result.n);
  lcd.print(F(" t="));
  lcd.print(result.t, 2);
  lcd.print(F("mm   "));
}

static void renderTimeout() {
  lcd.setCursor(0, 0);
  lcd.print(F("Sem resposta    "));
  lcd.setCursor(0, 1);
  lcd.print(F("SELECT p/ voltar"));
}

static void render() {
  switch (state) {
    case ST_INPUT:   renderInput();   break;
    case ST_WAIT:    renderWait();    break;
    case ST_RESULT:  renderResult();  break;
    case ST_TIMEOUT: renderTimeout(); break;
  }
}

// --------------------------------------------------------------------------
// Transicoes de estado por botao
// --------------------------------------------------------------------------
static void handleInput(Button b) {
  switch (b) {
    case BTN_LEFT:
    case BTN_RIGHT:
      field = (field == FIELD_QC) ? FIELD_TMAX : FIELD_QC;
      break;
    case BTN_UP:
      if (field == FIELD_QC) input.q_c   = clampf(input.q_c + QC_STEP, QC_MIN, QC_MAX);
      else                   input.t_max = clampf(input.t_max + TM_STEP, TM_MIN, TM_MAX);
      break;
    case BTN_DOWN:
      if (field == FIELD_QC) input.q_c   = clampf(input.q_c - QC_STEP, QC_MIN, QC_MAX);
      else                   input.t_max = clampf(input.t_max - TM_STEP, TM_MIN, TM_MAX);
      break;
    case BTN_SELECT:
      sendRequest();
      state = ST_WAIT;
      waitStart = millis();
      rxLen = 0;
      lcd.clear();
      break;
    default:
      break;
  }
}

static void handleButton(Button b) {
  if (b == BTN_NONE) return;
  switch (state) {
    case ST_INPUT:
      handleInput(b);
      break;
    case ST_WAIT:
      break;  // ignora botoes enquanto aguarda
    case ST_RESULT:
    case ST_TIMEOUT:
      if (b == BTN_SELECT) {   // volta para edicao
        state = ST_INPUT;
        lcd.clear();
      }
      break;
  }
}

// --------------------------------------------------------------------------
// setup / loop
// --------------------------------------------------------------------------
void setup() {
  pinMode(PIN_BACKLIGHT, OUTPUT);
  digitalWrite(PIN_BACKLIGHT, HIGH);
  Serial.begin(SERIAL_BAUD);
  lcd.begin(16, 2);
  lcd.clear();
}

void loop() {
  Button b = pollButton();
  handleButton(b);

  if (state == ST_WAIT) {
    serviceSerialRx();
    if (state == ST_WAIT && millis() - waitStart > REPLY_TIMEOUT) {
      state = ST_TIMEOUT;
      lcd.clear();
    }
  }

  render();
}
