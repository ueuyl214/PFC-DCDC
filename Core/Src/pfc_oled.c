#include "main.h"
#include "i2c.h"

#include <stddef.h>
#include <string.h>

#define PFC_OLED_WIDTH             128U
#define PFC_OLED_HEIGHT            64U
#define PFC_OLED_PAGE_COUNT        8U
#define PFC_OLED_FONT_WIDTH        6U
#define PFC_OLED_MAX_CHARS         (PFC_OLED_WIDTH / PFC_OLED_FONT_WIDTH)
#define PFC_OLED_I2C_TIMEOUT_MS    10U
#define PFC_OLED_REPROBE_MS        1000U

/* Default SSD1306 I2C address is 0x3C. Change to 0x3D if the module is strapped high. */
#ifndef PFC_OLED_I2C_ADDR_7BIT
#define PFC_OLED_I2C_ADDR_7BIT     0x3CU
#endif

#define PFC_OLED_I2C_ADDR_ALT_7BIT 0x3DU

static uint8_t oled_buffer[PFC_OLED_WIDTH * PFC_OLED_PAGE_COUNT];
static uint8_t oled_i2c_addr = (uint8_t)(PFC_OLED_I2C_ADDR_7BIT << 1U);
static uint8_t oled_ready = 0U;
static uint8_t oled_probe_failed = 0U;
static uint32_t oled_next_probe_ms = 0U;

static HAL_StatusTypeDef PFC_OLED_SendCommand(uint8_t command)
{
  uint8_t tx[2] = {0x00U, command};
  return HAL_I2C_Master_Transmit(&hi2c1, oled_i2c_addr, tx, sizeof(tx), PFC_OLED_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef PFC_OLED_SendCommands(const uint8_t *commands, size_t count)
{
  size_t i;

  for (i = 0U; i < count; i++)
  {
    if (PFC_OLED_SendCommand(commands[i]) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}

static void PFC_OLED_SelectAddress(void)
{
  const uint8_t preferred = (uint8_t)(PFC_OLED_I2C_ADDR_7BIT << 1U);
  const uint8_t alternate = (uint8_t)(PFC_OLED_I2C_ADDR_ALT_7BIT << 1U);

  if (HAL_I2C_IsDeviceReady(&hi2c1, preferred, 1U, PFC_OLED_I2C_TIMEOUT_MS) == HAL_OK)
  {
    oled_i2c_addr = preferred;
    oled_probe_failed = 0U;
    return;
  }

  if (HAL_I2C_IsDeviceReady(&hi2c1, alternate, 1U, PFC_OLED_I2C_TIMEOUT_MS) == HAL_OK)
  {
    oled_i2c_addr = alternate;
    oled_probe_failed = 0U;
    return;
  }

  oled_probe_failed = 1U;
}

static void PFC_OLED_InitIfNeeded(void)
{
  static const uint8_t init_cmds[] = {
    0xAEU,       /* display off */
    0x20U, 0x02U, /* page addressing: one 6x8 text row per SSD1306 page */
    0xB0U,
    0xC8U,       /* COM scan direction remapped */
    0x00U,
    0x10U,
    0x40U,
    0x81U, 0x7FU,
    0xA1U,       /* segment remap */
    0xA6U,       /* normal display */
    0xA8U, 0x3FU,
    0xA4U,
    0xD3U, 0x00U,
    0xD5U, 0x80U,
    0xD9U, 0xF1U,
    0xDAU, 0x12U,
    0xDBU, 0x40U,
    0x8DU, 0x14U, /* charge pump on */
    0x2EU,       /* deactivate scroll */
    0xAFU        /* display on */
  };
  const uint32_t now_ms = HAL_GetTick();

  if (oled_ready != 0U)
  {
    return;
  }

  if ((oled_probe_failed != 0U) && (now_ms < oled_next_probe_ms))
  {
    return;
  }

  PFC_OLED_SelectAddress();
  if (oled_probe_failed != 0U)
  {
    oled_next_probe_ms = now_ms + PFC_OLED_REPROBE_MS;
    return;
  }

  if (PFC_OLED_SendCommands(init_cmds, sizeof(init_cmds)) == HAL_OK)
  {
    oled_ready = 1U;
    memset(oled_buffer, 0, sizeof(oled_buffer));
  }
  else
  {
    oled_probe_failed = 1U;
    oled_next_probe_ms = now_ms + PFC_OLED_REPROBE_MS;
  }
}

static void PFC_OLED_GetGlyph(char ch, uint8_t glyph[5])
{
  static const uint8_t blank[5] = {0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
  const uint8_t *src = blank;

  switch (ch)
  {
    case '0': { static const uint8_t g[5] = {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU}; src = g; break; }
    case '1': { static const uint8_t g[5] = {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U}; src = g; break; }
    case '2': { static const uint8_t g[5] = {0x42U, 0x61U, 0x51U, 0x49U, 0x46U}; src = g; break; }
    case '3': { static const uint8_t g[5] = {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U}; src = g; break; }
    case '4': { static const uint8_t g[5] = {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U}; src = g; break; }
    case '5': { static const uint8_t g[5] = {0x27U, 0x45U, 0x45U, 0x45U, 0x39U}; src = g; break; }
    case '6': { static const uint8_t g[5] = {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U}; src = g; break; }
    case '7': { static const uint8_t g[5] = {0x01U, 0x71U, 0x09U, 0x05U, 0x03U}; src = g; break; }
    case '8': { static const uint8_t g[5] = {0x36U, 0x49U, 0x49U, 0x49U, 0x36U}; src = g; break; }
    case '9': { static const uint8_t g[5] = {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU}; src = g; break; }
    case 'A': { static const uint8_t g[5] = {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU}; src = g; break; }
    case 'B': { static const uint8_t g[5] = {0x7FU, 0x49U, 0x49U, 0x49U, 0x36U}; src = g; break; }
    case 'C': { static const uint8_t g[5] = {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U}; src = g; break; }
    case 'D': { static const uint8_t g[5] = {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU}; src = g; break; }
    case 'E': { static const uint8_t g[5] = {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U}; src = g; break; }
    case 'F': { static const uint8_t g[5] = {0x7FU, 0x09U, 0x09U, 0x09U, 0x01U}; src = g; break; }
    case 'G': { static const uint8_t g[5] = {0x3EU, 0x41U, 0x49U, 0x49U, 0x7AU}; src = g; break; }
    case 'H': { static const uint8_t g[5] = {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU}; src = g; break; }
    case 'I': { static const uint8_t g[5] = {0x00U, 0x41U, 0x7FU, 0x41U, 0x00U}; src = g; break; }
    case 'K': { static const uint8_t g[5] = {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U}; src = g; break; }
    case 'L': { static const uint8_t g[5] = {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U}; src = g; break; }
    case 'M': { static const uint8_t g[5] = {0x7FU, 0x02U, 0x0CU, 0x02U, 0x7FU}; src = g; break; }
    case 'N': { static const uint8_t g[5] = {0x7FU, 0x04U, 0x08U, 0x10U, 0x7FU}; src = g; break; }
    case 'O': { static const uint8_t g[5] = {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU}; src = g; break; }
    case 'P': { static const uint8_t g[5] = {0x7FU, 0x09U, 0x09U, 0x09U, 0x06U}; src = g; break; }
    case 'R': { static const uint8_t g[5] = {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U}; src = g; break; }
    case 'S': { static const uint8_t g[5] = {0x46U, 0x49U, 0x49U, 0x49U, 0x31U}; src = g; break; }
    case 'T': { static const uint8_t g[5] = {0x01U, 0x01U, 0x7FU, 0x01U, 0x01U}; src = g; break; }
    case 'U': { static const uint8_t g[5] = {0x3FU, 0x40U, 0x40U, 0x40U, 0x3FU}; src = g; break; }
    case 'V': { static const uint8_t g[5] = {0x1FU, 0x20U, 0x40U, 0x20U, 0x1FU}; src = g; break; }
    case 'W': { static const uint8_t g[5] = {0x3FU, 0x40U, 0x38U, 0x40U, 0x3FU}; src = g; break; }
    case 'Y': { static const uint8_t g[5] = {0x07U, 0x08U, 0x70U, 0x08U, 0x07U}; src = g; break; }
    case '_': { static const uint8_t g[5] = {0x40U, 0x40U, 0x40U, 0x40U, 0x40U}; src = g; break; }
    case ':': { static const uint8_t g[5] = {0x00U, 0x36U, 0x36U, 0x00U, 0x00U}; src = g; break; }
    case '.': { static const uint8_t g[5] = {0x00U, 0x60U, 0x60U, 0x00U, 0x00U}; src = g; break; }
    case '-': { static const uint8_t g[5] = {0x08U, 0x08U, 0x08U, 0x08U, 0x08U}; src = g; break; }
    case '%': { static const uint8_t g[5] = {0x23U, 0x13U, 0x08U, 0x64U, 0x62U}; src = g; break; }
    case 'a': { static const uint8_t g[5] = {0x20U, 0x54U, 0x54U, 0x54U, 0x78U}; src = g; break; }
    case 'r': { static const uint8_t g[5] = {0x7CU, 0x08U, 0x04U, 0x04U, 0x08U}; src = g; break; }
    case 'w': { static const uint8_t g[5] = {0x3CU, 0x40U, 0x30U, 0x40U, 0x3CU}; src = g; break; }
    default:
      src = blank;
      break;
  }

  memcpy(glyph, src, 5U);
}

void PFC_OLED_Clear(void)
{
  PFC_OLED_InitIfNeeded();

  if (oled_ready == 0U)
  {
    return;
  }

  memset(oled_buffer, 0, sizeof(oled_buffer));
}

void PFC_OLED_WriteLine(uint8_t line, const char *text)
{
  uint8_t x = 0U;

  PFC_OLED_InitIfNeeded();

  if ((oled_ready == 0U) || (line >= PFC_OLED_PAGE_COUNT) || (text == NULL))
  {
    return;
  }

  memset(&oled_buffer[(uint16_t)line * PFC_OLED_WIDTH], 0, PFC_OLED_WIDTH);

  while ((text[x] != '\0') && (x < PFC_OLED_MAX_CHARS))
  {
    uint8_t glyph[5];
    uint8_t col;
    const uint16_t dst = ((uint16_t)line * PFC_OLED_WIDTH) + ((uint16_t)x * PFC_OLED_FONT_WIDTH);

    PFC_OLED_GetGlyph(text[x], glyph);
    for (col = 0U; col < 5U; col++)
    {
      oled_buffer[dst + col] = glyph[col];
    }
    oled_buffer[dst + 5U] = 0x00U;

    x++;
  }
}

void PFC_OLED_Update(void)
{
  uint8_t page;

  PFC_OLED_InitIfNeeded();

  if (oled_ready == 0U)
  {
    return;
  }

  for (page = 0U; page < PFC_OLED_PAGE_COUNT; page++)
  {
    uint8_t x;

    (void)PFC_OLED_SendCommand((uint8_t)(0xB0U + page));
    (void)PFC_OLED_SendCommand(0x00U);
    (void)PFC_OLED_SendCommand(0x10U);

    for (x = 0U; x < PFC_OLED_WIDTH; x += 16U)
    {
      uint8_t tx[17];
      tx[0] = 0x40U;
      memcpy(&tx[1], &oled_buffer[((uint16_t)page * PFC_OLED_WIDTH) + x], 16U);

      if (HAL_I2C_Master_Transmit(&hi2c1, oled_i2c_addr, tx, sizeof(tx), PFC_OLED_I2C_TIMEOUT_MS) != HAL_OK)
      {
        oled_ready = 0U;
        oled_probe_failed = 1U;
        oled_next_probe_ms = HAL_GetTick() + PFC_OLED_REPROBE_MS;
        return;
      }
    }
  }
}
