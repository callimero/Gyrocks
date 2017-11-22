/*
   Vector Game "Gyrocks" auf dem Oszilloskop
   Carsten Wartmann 2016/2017 cw@blenderbuch.de
   Fürs Make-Magazin

   Gehackt und basierend auf Trammel Hudsons Arbeit:

   Vector display using the MCP4921 DAC on the teensy3.1.
   More info: https://trmm.net/V.st
*/

/*
   Todo:
   - Alles auf ein System/Skalierung umstellen, fixe integer Mathe
   - implement Bounding Box Collision
   + Enemies und Rocks trennen
   + weniger Schüsse gleichzeitig
   - Enemy und Rocks können Ship schaden
   - Bug in Collisions Erkennung?
    - Enemies schießen
   - Lebenszähler für Ship
   + einfache Punktezählung
   - Feinde Formationen fliegen lassen
   - Alles auf Polarkoord umstellen (Ship/Bullets) für Kollisionsabfrage
   - Explosionen

*/

#include <SPI.h>
#include "DMAChannel.h"
#include <math.h>

#include "hershey_font.h"
#include "objects.h"

//#define CONFIG_VECTREX
#define CONFIG_VECTORSCOPE

// Sometimes the X and Y need to be flipped and/or swapped
#undef FLIP_X
#undef FLIP_Y
//#define SWAP_XY


#if defined(CONFIG_VECTORSCOPE)
/** Vectorscope configuration.

   Vectorscopes and oscilloscopes have electrostatic beam deflection
   and can steer the beam much faster, so there is no need to dwell or
   wait for the beam to catch up during transits.

   Most of them do not have a Z input, so we move the beam to an extreme
   during the blanking interval.

   If your vectorscope doesn't have a Z axis, undefine CONFIG_BRIGHTNESS
*/
#define BRIGHT_SHIFT	0	// larger numbers == dimmer lines
#define NORMAL_SHIFT	1	// no z-axis, so we must have a difference
#define OFF_JUMP		// don't wait for beam, just go!
#define REST_X		0	// wait off screen
#define REST_Y		0

#define FULL_SCALE		// only use -1.25 to 1.25V range

// most vector scopes don't have brightness control, but set it anyway
#define CONFIG_BRIGHTNESS
#define BRIGHT_OFF	2048	// "0 volts", relative to reference
#define BRIGHT_NORMAL	3800	// lowest visible
#define BRIGHT_BRIGHT	4095	// super bright
#define OFF_DWELL0  10  // time to sit beam on before starting a transit


#elif defined(CONFIG_VECTREX)
/** Vectrex configuration.

   Vectrex monitors use magnetic steering and are much slower to respond
   to changes.  As a result we must dwell on the end points and wait for
   the beam to catch up during transits.

   It does have a Z input for brightness, which has three configurable
   brightnesses (off, normal and bright).  These were experimentally
   determined and might not be right for all monitors.
*/

#define BRIGHT_SHIFT	2	// larger numbers == dimmer lines
#define NORMAL_SHIFT	2	// but we can control with Z axis
#undef OFF_JUMP			// too slow, so we can't jump the beam

#define OFF_SHIFT	5	// smaller numbers == slower transits
#define OFF_DWELL0	10	// time to sit beam on before starting a transit
#define OFF_DWELL1	0	// time to sit before starting a transit
#define OFF_DWELL2	10	// time to sit after finishing a transit

#define REST_X		2048	// wait in the center of the screen
#define REST_Y		2048

#define CONFIG_BRIGHTNESS	// use the brightness DAC
#define BRIGHT_OFF	2048	// "0 volts", relative to reference
#define BRIGHT_NORMAL	3200	// fairly bright
#define BRIGHT_BRIGHT	4095	// super bright

#define FULL_SCALE		// use the full -2.5 to 2.5V range

#else
#error "One of CONFIG_VECTORSCOPE or CONFIG_VECTREX must be defined"
#endif


// Wichtige Pins wie am Teensy->DAC angeschlossen
#define SS_PIN	10  // Chip Select 2
#define SS2_PIN	6   // Chip Select 1
#define SDI	11      // 
#define SCK	13



#define MAX_PTS 2000
static unsigned rx_points;
static unsigned num_points;
static uint32_t points[MAX_PTS];

#define MOVETO		(1<<11)
#define LINETO		(2<<11)
#define BRIGHTTO	(3<<11)
#undef LINE_BRIGHT_DOUBLE


static DMAChannel spi_dma;
#define SPI_DMA_MAX 4096
//#define SPI_DMA_MAX 2048 // ??
static uint32_t spi_dma_q[2][SPI_DMA_MAX];
static unsigned spi_dma_which;
static unsigned spi_dma_count;
static unsigned spi_dma_in_progress;
static unsigned spi_dma_cs; // which pins are we using for IO

#define SPI_DMA_CS_BEAM_ON 2
#define SPI_DMA_CS_BEAM_OFF 1

// x and y position are in 12-bit range
static uint16_t x_pos;
static uint16_t y_pos;

#ifdef SWAP_XY
#define DAC_X_CHAN 1
#define DAC_Y_CHAN 0
#else
#define DAC_X_CHAN 0
#define DAC_Y_CHAN 1
#endif


/*  *********************** Game Stuff ***************************************************/

// Rock
typedef struct
{
  int16_t t = -1;
  int16_t r = 0;
  int16_t p = 0;
  int16_t x = 0;    // x/y Merker für Kollisionsabfrage
  int16_t y = 0;
  int16_t d = 0;
  int16_t vr;
  int16_t vp;
} rock_t;

// Enemy
typedef struct
{
  int16_t t = -1;
  int16_t r = 0;
  int16_t p = 0;
  int16_t vr;
  int16_t vp;
} enemy_t;

// Ship
typedef struct
{
  int16_t x = 1000;
  int16_t y = 1000;
  int16_t ax;
  int16_t ay;
  unsigned long firedelay;
} ship_t;

// Star
typedef struct
{
  int16_t x;
  int16_t y;
  int16_t vx;
  int16_t vy;
  int16_t age;
} star_t;

// Bullet
typedef struct
{
  int16_t x;
  int16_t y;
  int16_t rot;
  int16_t vx;
  int16_t vy;
  int16_t age = -1;
} bullet_t;


#define HALT  // Auskommentieren um "Handbremse" für zweiten Knopf/Schalter zu lösen (Debug&Screenshot)

// Joystick
#define BUTT 14   // Digital
#define TRIG 15   // Digital
#define THRU 16    // Analog
#define POTX 17   // Analog
#define POTY 18    // Analog
#define DEADX 30  // Deadband X
#define DEADY 30  // Deadband Y

#define FIREDELAY 100   // Zweitverzögerung zwischen zwei Schüssen

// Hintergrundsterne
#define MAX_STARS 30
star_t s[MAX_STARS];

// max. Anzahl der Schüsse
#define MAX_BULLETS 5
bullet_t b[MAX_BULLETS];

// max. Zahl der Asteroiden/Rocks
#define MAX_ROCK 5
rock_t r[MAX_ROCK];

// max. Zahl der Feinde
#define MAX_ENEMY 5
enemy_t e[MAX_ENEMY];

// Infos zum Schiff/Ship speichern
ship_t ship;

// Punktezähler
unsigned int score;

// Frames per Second/Framerate Merker
long fps;

// Schnelle aber ungenaue Sinus Tabelle
const  uint8_t isinTable8[] = {
  0, 4, 9, 13, 18, 22, 27, 31, 35, 40, 44,
  49, 53, 57, 62, 66, 70, 75, 79, 83, 87,
  91, 96, 100, 104, 108, 112, 116, 120, 124, 128,

  131, 135, 139, 143, 146, 150, 153, 157, 160, 164,
  167, 171, 174, 177, 180, 183, 186, 190, 192, 195,
  198, 201, 204, 206, 209, 211, 214, 216, 219, 221,

  223, 225, 227, 229, 231, 233, 235, 236, 238, 240,
  241, 243, 244, 245, 246, 247, 248, 249, 250, 251,
  252, 253, 253, 254, 254, 254, 255, 255, 255, 255,
};

// Schnelle aber ungenaue Sinus Funktion 
int isin(int x)
{
  boolean pos = true;  // positive - keeps an eye on the sign.
  uint8_t idx;
  // remove next 6 lines for fastest execution but without error/wraparound
  if (x < 0)
  {
    x = -x;
    pos = !pos;
  }
  if (x >= 360) x %= 360;
  if (x > 180)
  {
    idx = x - 180;
    pos = !pos;
  }
  else idx = x;
  if (idx > 90) idx = 180 - idx;
  if (pos) return isinTable8[idx] / 2 ;
  return -(isinTable8[idx] / 2);
}

// Cosinus
int icos(int x)
{
  return (isin(x + 90));
}




/* ************************************** DAC/vektor output stuff **************************************/
static int
spi_dma_tx_append(
  uint16_t value
)
{
  spi_dma_q[spi_dma_which][spi_dma_count++] = 0
      | ((uint32_t) value)
      | (spi_dma_cs << 16) // enable the chip select line
      ;

  if (spi_dma_count == SPI_DMA_MAX)
    return 1;
  return 0;
}


static void
spi_dma_tx()
{
  if (spi_dma_count == 0)
    return;

  //  digitalWriteFast(DELAY_PIN, 1);

  // add a EOQ to the last entry
  spi_dma_q[spi_dma_which][spi_dma_count - 1] |= (1 << 27);

  spi_dma.clearComplete();
  spi_dma.clearError();
  spi_dma.sourceBuffer(
    spi_dma_q[spi_dma_which],
    4 * spi_dma_count  // in bytes, not thingies
  );

  spi_dma_which = !spi_dma_which;
  spi_dma_count = 0;

  SPI0_SR = 0xFF0F0000;
  SPI0_RSER = 0
              | SPI_RSER_RFDF_RE
              | SPI_RSER_RFDF_DIRS
              | SPI_RSER_TFFF_RE
              | SPI_RSER_TFFF_DIRS;

  spi_dma.enable();
  spi_dma_in_progress = 1;
}


static int spi_dma_tx_complete()
{
  //cli();

  // if nothing is in progress, we're "complete"
  if (!spi_dma_in_progress)
  {
    //sei();
    return 1;
  }

  if (!spi_dma.complete())
  {
    //sei();
    return 0;
  }

  //  digitalWriteFast(DELAY_PIN, 0);

  spi_dma.clearComplete();
  spi_dma.clearError();

  // the DMA hardware lies; it is not actually complete
  delayMicroseconds(5);
  //sei();

  // we are done!
  SPI0_RSER = 0;
  SPI0_SR = 0xFF0F0000;
  spi_dma_in_progress = 0;
  return 1;
}


static void
spi_dma_setup()
{
  spi_dma.disable();
  spi_dma.destination((volatile uint32_t&) SPI0_PUSHR);
  spi_dma.disableOnCompletion();
  spi_dma.triggerAtHardwareEvent(DMAMUX_SOURCE_SPI0_TX);
  spi_dma.transferSize(4); // write all 32-bits of PUSHR

  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));

  // configure the output on pin 10 for !SS0 from the SPI hardware
  // and pin 6 for !SS1.
  CORE_PIN10_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2);
  CORE_PIN6_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2);

  // configure the frame size for 16-bit transfers
  SPI0_CTAR0 |= 0xF << 27;

  // send something to get it started

  spi_dma_which = 0;
  spi_dma_count = 0;

  spi_dma_tx_append(0);
  spi_dma_tx_append(0);
  spi_dma_tx();
}


void rx_append(int x, int y, unsigned bright)
{
  // store the 12-bits of x and y, as well as 6 bits of brightness
  // (three in X and three in Y)
  points[rx_points++] = (bright << 24) | x << 12 | y << 0;
}


void moveto(int x, int y)
{
  //Test!  Very stupid "Clipping"
  if (x >= 4096) x = 4095;
  if (y >= 4096) y = 4095;
  if (x < 0) x = 0;
  if (y < 0) y = 0;
  rx_append(x, y, 0);
}


void lineto(int x, int y)
{
  //Test!  Very stupid "Clipping"
  //if (x>=4096 ||x<0 ||y>4096 || y<0) return; //don't draw at all
  if (x >= 4096) x = 4095;
  if (y >= 4096) y = 4095;
  if (x < 0) x = 0;
  if (y < 0) y = 0;

  rx_append(x, y, 24); // normal brightness
}


void brightto(int x, int y)
{
  rx_append(x, y, 63); // max brightness
}


int draw_character(char c, int x, int y, int size)
{
  const hershey_char_t * const f = &hershey_simplex[c - ' '];
  int next_moveto = 1;

  for (int i = 0 ; i < f->count ; i++)
  {
    int dx = f->points[2 * i + 0];
    int dy = f->points[2 * i + 1];
    if (dx == -1)
    {
      next_moveto = 1;
      continue;
    }
    dx = (dx * size) * 3 / 4;
    dy = (dy * size) * 3 / 4; //??
    if (next_moveto)
      moveto(x + dx, y + dy);
    else
      lineto(x + dx, y + dy);
    next_moveto = 0;
  }
  return (f->width * size) * 3 / 4;
}


void draw_string(const char * s, int x, int y, int size)
{
  while (*s)
  {
    char c = *s++;
    x += draw_character(c, x, y, size);
  }
}


static void mpc4921_write(int channel,  uint16_t value)
{
  value &= 0x0FFF; // mask out just the 12 bits of data
#if 1       //???
  // select the output channel, buffered, no gain
  value |= 0x7000 | (channel == 1 ? 0x8000 : 0x0000);
#else
  // select the output channel, unbuffered, no gain
  value |= 0x3000 | (channel == 1 ? 0x8000 : 0x0000);
#endif

#ifdef SLOW_SPI
  SPI.transfer((value >> 8) & 0xFF);
  SPI.transfer((value >> 0) & 0xFF);
#else
  if (spi_dma_tx_append(value) == 0)
    return;
  // wait for the previous line to finish
  while (!spi_dma_tx_complete())
    ;
  // now send this line, which swaps buffers
  spi_dma_tx();
#endif
}


static inline void goto_x(uint16_t x)
{
  x_pos = x;
#ifdef FLIP_X
  mpc4921_write(DAC_X_CHAN, 4095 - x);
#else
  mpc4921_write(DAC_X_CHAN, x);
#endif
}


static inline void goto_y(uint16_t y)
{
  y_pos = y;
#ifdef FLIP_Y
  mpc4921_write(DAC_Y_CHAN, 4095 - y);
#else
  mpc4921_write(DAC_Y_CHAN, y);
#endif
}


static void dwell(const int count)
{
  for (int i = 0 ; i < count ; i++)
  {
    if (i & 1)
      goto_x(x_pos);
    else
      goto_y(y_pos);
  }
}


static inline void brightness(uint16_t bright)
{
#ifdef CONFIG_BRIGHTNESS
  static unsigned last_bright;
  if (last_bright == bright)
    return;
  last_bright = bright;

  dwell(OFF_DWELL0);
  spi_dma_cs = SPI_DMA_CS_BEAM_OFF;

  // scale bright from OFF to BRIGHT
  if (bright > 64)
    bright = 64;

  int bright_scaled = BRIGHT_OFF;
  if (bright > 0)
    bright_scaled = BRIGHT_NORMAL + ((BRIGHT_BRIGHT - BRIGHT_NORMAL) * bright) / 64;

  mpc4921_write(0, bright_scaled);
  spi_dma_cs = SPI_DMA_CS_BEAM_ON;
#else
  (void) bright;
#endif
}


static inline void _draw_lineto(int x1, int y1, const int bright_shift)
{
  int dx;
  int dy;
  int sx;
  int sy;

  const int x1_orig = x1;
  const int y1_orig = y1;

  int x_off = x1 & ((1 << bright_shift) - 1);
  int y_off = y1 & ((1 << bright_shift) - 1);
  x1 >>= bright_shift;
  y1 >>= bright_shift;
  int x0 = x_pos >> bright_shift;
  int y0 = y_pos >> bright_shift;

  goto_x(x_pos);
  goto_y(y_pos);

  if (x0 <= x1)
  {
    dx = x1 - x0;
    sx = 1;
  } else {
    dx = x0 - x1;
    sx = -1;
  }

  if (y0 <= y1)
  {
    dy = y1 - y0;
    sy = 1;
  } else {
    dy = y0 - y1;
    sy = -1;
  }
  int err = dx - dy;

  while (1)
  {
    if (x0 == x1 && y0 == y1)
      break;

    int e2 = 2 * err;
    if (e2 > -dy)
    {
      err = err - dy;
      x0 += sx;
      goto_x(x_off + (x0 << bright_shift));
    }
    if (e2 < dx)
    {
      err = err + dx;
      y0 += sy;
      goto_y(y_off + (y0 << bright_shift));
    }

#ifdef LINE_BRIGHT_DOUBLE
    if (bright_shift == 0)
    {
      goto_x(x_off + (x0 << bright_shift));
      goto_y(y_off + (y0 << bright_shift));
    }
#endif
  }

  // ensure that we end up exactly where we want or don't care
  //  goto_x(x1_orig);
  //  goto_y(y1_orig);
}


void draw_lineto(int x1, int y1, unsigned bright)
{
  brightness(bright);
  _draw_lineto(x1, y1, NORMAL_SHIFT);
}


void draw_moveto(int x1, int y1)
{
  brightness(0);
#ifdef OFF_JUMP
  goto_x(x1);
  goto_y(y1);
#else
  // hold the current position for a few clocks
  // with the beam off
  dwell(OFF_DWELL1);
  _draw_lineto(x1, y1, OFF_SHIFT);
  dwell(OFF_DWELL2);
#endif // OFF_JUMP
}





/* Setup all */
void setup()
{
  pinMode(SS_PIN, OUTPUT);
  pinMode(SS2_PIN, OUTPUT);
  pinMode(SDI, OUTPUT);
  pinMode(SCK, OUTPUT);

  // Joystick
  pinMode(BUTT, INPUT);
  pinMode(TRIG, INPUT);


  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  spi_dma_setup();

  //Serial.begin(9600); // For Debugging
  init_stars(s);
}



/* ***************************** Game Stuff **************************************************/

// Ähnlich draw_string aber mit definierter Rotation
void draw_object(byte c, int x, int y, int size, int rot)
{
  const objects_char_t * const f = &gobjects[c];

  int next_moveto = 1;
  int dxx, dyy;

  for (int i = 0 ; i < f->count ; i++)
  {
    int dx = f->points[2 * i + 0];
    int dy = f->points[2 * i + 1];
    if (dx == -127)
    {
      next_moveto = 1;
      continue;
    }
    dxx = ((dx * icos(rot)) - dy * isin(rot)) >> 7 ; // Würg irgendwie nicht Standard, KoordSys komisch?
    dyy = ((dy * icos(rot)) + dx * isin(rot)) >> 7 ;

    dx = (dxx * size) * 3 / 4 ;
    dy = (dyy * size) ;

    if (next_moveto)
      moveto(x + dx, y + dy);
    else
      lineto(x + dx, y + dy);
    next_moveto = 0;
  }
}


// Neuer Schuß wenn eine Slot frei (age==-1)
static void add_bullet(bullet_t * const bullets, ship_t * const ship, int rot)
{
  for (uint8_t i = 0 ; i < MAX_BULLETS ; i++)
  {
    bullet_t * const b = &bullets[i];
    if (b->age < 0)
    {
      b->x = ship->x;
      b->y = ship->y;

      b->rot = rot;
      b->vx = ship->ax ;
      b->vy = ship->ay ;
      b->age = 1;
      break;
    }
  }
}


// Updating bullets/Schüsse
static void update_bullets(bullet_t * const bullets)
{
  for (uint8_t i = 0 ; i < MAX_BULLETS ; i++)
  {
    bullet_t * const b = &bullets[i];
    if (b->age >= 0)
    {
      if (b->age > 100 || (b->x > 2000 - 96 && b->x < 2000 + 96 && b->y > 2000 - 96 && b->y < 2000 + 96))
      {
        b->age = -1;
      }
      else
      {
        b->age++;
        b->x = b->x + (b->vx >> 1);
        b->y = b->y + (b->vy >> 1);
        draw_object(6, b->x, b->y, 10, b->rot);
      }
    }
  }
}


// Schiff Verwaltung
static void update_ship(ship_t * const ship)
{
  long d;
  int rot;

  d = ((2048 - ship->x) * (2048 - ship->x) + (2048 - ship->y) * (2048 - ship->y)) / 75000;
  if (d > 15) d = 15;
  if (d < 1) d = 1;

  if (collision_rock(ship->x, ship->y, d))
  {
    score = 0;
  }

  rot = atan2(2048 - ship->y, 2048 - ship->x) * 180.0 / PI - 90;  // different coord sys...?! Float... hmm

  // Fire
  if (!digitalRead(TRIG) == HIGH && millis() > (ship->firedelay + FIREDELAY))
  {
    ship->firedelay = millis();
    ship->ax = -isin(rot) >> 1  ;
    ship->ay =  icos(rot) >> 1  ;
    add_bullet(b, ship, rot);
  }
  draw_object(3, ship->x, ship->y, d, rot);               // Ship
  draw_object(4, ship->x, ship->y, d + rand() % d, rot);  // Engine
}


// Hintergrund
static void  update_stars(star_t * const stars)
{
  int age2;
  for (uint8_t i = 0 ; i < MAX_STARS ; i++)
  {
    star_t * const s = &stars[i];
    s->age++;
    age2 = s->age * s->age >> 12;
    s->x = s->x + (s->vx * age2);
    s->y = s->y + (s->vy * age2);
    if (s->x > 4000 || s->x < 96 || s->y > 4000 || s->y < 96 || s->age > 200)
    {
      s->x = rand() % 50 + 2000;
      s->y = rand() % 50 + 2000;
      s->vx = rand() % 8 - 4 ;
      s->vy = rand() % 8 - 4 ;
      s->age = 0;
    }
    draw_character(43, s->x, s->y, age2 >> 1);  // Using a "+" char...
  }
}

static void  init_stars(star_t * const stars)
{
  for (uint8_t i = 0 ; i < MAX_STARS ; i++)
  {
    star_t * const s = &stars[i];
    s->x = rand() % 500 + 1750;
    s->y = rand() % 500 + 1750;

    s->vx = rand() % 8 - 4 ;
    s->vy = rand() % 8 - 4 ;
    s->age = rand() % 300;
  }
}


// Felsen/Rock/Asteroid
static void  add_rock(rock_t * const rock)
{
  for (uint8_t i = 0 ; i < MAX_ROCK ; i++)
  {
    rock_t * const rr = &rock[i];
    if (rr->t == -1)
    {
      rr->t = rand() % 2 + 13;
      rr->r = rand() % 1000 + 1;
      rr->p = rand() % 359 * 16;

      rr->vr = rand() % 30 + 15;
      rr->vp = rand() % 10 - 5  ;
      break;
    }
  }
}

static void  update_rocks(rock_t * const rr)
{
  int x, y;
  for (uint8_t i = 0 ; i < MAX_ROCK ; i++)
  {
    //rock_t * const rr = &rock[i];
    if (rr[i].t >= 0) // nur wenn live/type gesetzt
    {
      if (rr[i].r < 30000 )
      {
        rr[i].r += rr[i].vr;
      }
      else
      {
        rr[i].t = -1;
        continue;
      }
      rr[i].p = (rr[i].p + rr[i].vp) % (360 * 16) ;
      x = 2048 + (rr[i].r / 16 * icos(rr[i].p / 16)) / 100;
      y = 2048 + (rr[i].r / 16 * isin(rr[i].p / 16)) / 100;
      rr[i].x = x;
      rr[i].y = y;  // Keep track, x,y ToDo: raus oder auf Polarkoords
      rr[i].d = rr[i].r / 512;

      if (collision_bullet(x, y, rr[i].r / 512))
      {
        rr[i].t = -1;
        rr[i].r = 0;
        score += 10;
      }
      else
      {
        draw_object(rr[i].t, x, y, rr[i].d, -rr[i].p / 4);
        draw_rect(r[i].x - r[i].d * 6, r[i].y - r[i].d * 6, r[i].x + r[i].d * 6, r[i].y + r[i].d * 6); // debug
      }
    }
  }
}


// Enemies
static void  add_enemy(enemy_t * const enemy)
{
  for (uint8_t i = 0 ; i < MAX_ENEMY ; i++)
  {
    enemy_t * const rr = &enemy[i];
    if (rr->t == -1)
    {
      rr->t = rand() % 4 + 18;
      rr->r = rand() % 1000 + 1;
      rr->p = rand() % 359 * 16;

      rr->vr = rand() % 30 + 15;
      rr->vp = rand() % 10 - 5  ;
      break;
    }
  }
}

static void  update_enemies(enemy_t * const rr)
{
  int x, y;
  for (uint8_t i = 0 ; i < MAX_ENEMY ; i++)
  {
    if (rr[i].t >= 0) // nur wenn live/type gesetzt
    {
      if (rr[i].r < 10000 )
      {
        rr[i].r += rr[i].vr;
      }

      rr[i].p = (rr[i].p + rr[i].vp) % (360 * 16) ;
      x = 2048 + (rr[i].r / 16 * icos(rr[i].p / 16)) / 100;
      y = 2048 + (rr[i].r / 16 * isin(rr[i].p / 16)) / 100;

      if (collision_bullet(x, y, rr[i].r / 512))
      {
        rr[i].t = -1;
        rr[i].r = 0;
        score += 100;
      }
      else
      {
        draw_object(rr[i].t, x, y, rr[i].r / 512, -rr[i].p / 4);
      }
    }
  }
}


// Irgndwie verallgemeinern?
int collision_rock(int x, int y, int d)
{
  int x0, y0, x1, y1;

  d = d * 10;
  x0 = x - d;
  y0 = y - d ;
  x1 = x + d;
  y1 = y + d ;

  draw_rect(x0, y0, x1, y1);
  for (uint8_t i = 0 ; i < MAX_ROCK ; i++)
  {
    if (r[i].t >= 0)
    {
      if (r[i].x > x0 && r[i].x < x1 && r[i].y > y0 && r[i].y < y1)
      {
        //        r[i].t = -1;    //Kill rock also?
        return 1; //Collision with Bullet
      }
    }
  }
  return 0; // No Collision
}



int collision_bullet(int x, int y, int d)
{
  int x0, y0, x1, y1;

  d = d * 10;
  x0 = x - d / 2;
  y0 = y - d / 2;
  x1 = x + d / 2;
  y1 = y + d / 2;

  for (uint8_t i = 0 ; i < MAX_BULLETS ; i++)
  {
    //    bullet_t * const b = &bullets[i];
    if (b[i].age >= 0)
    {
      if (b[i].x > x0 && b[i].x < x1 && b[i].y > y0 && b[i].y < y1)
      {
        b[i].age = -1;
        return 1; //Kollision mit Schuss/Bullet
      }
    }
  }
  return 0; // Keine Kollision
}


void draw_field()
{
#define CORNER 500
  moveto(0, CORNER);
  lineto(0, 0);
  lineto(CORNER, 0);

  moveto(4095 - CORNER, 0);
  lineto(4095, 0);
  lineto(4095, CORNER);

  moveto(4095, 4095 - CORNER);
  lineto(4095, 4095);
  lineto(4095 - CORNER, 4095);

  moveto(CORNER, 4095);
  lineto(0, 4095);
  lineto(0, 4095 - CORNER);
}


// Debug draw
void draw_rect(int x0, int y0, int x1, int y1)
{
  return;    //Debug!
  moveto(x0, y0);
  lineto(x1, y0);
  lineto(x1, y1);
  lineto(x0, y1);
  lineto(x0, y0);
}



// Anzeige Funktion
void video()
{
  // Joystick auslesen
  if (analogRead(POTX) > 512 + DEADX || analogRead(POTX) < 512 - DEADX)
  {
    ship.x = ship.x - (analogRead(POTX) - 512) / 4;
  }
  if (analogRead(POTY) > 512 + DEADY || analogRead(POTY) < 512 - DEADY)
  {
    ship.y = ship.y - (analogRead(POTY) - 512) / 4;
  }

  ship.x = constrain(ship.x, 400, 3700);
  ship.y = constrain(ship.y, 400, 3700);

  update_stars(s);
  update_bullets(b);
  update_rocks(r);
  if (rand() % 500 == 1) add_rock(r);
  update_enemies(e);
  if (rand() % 500 == 1) add_enemy(e);
  update_ship(&ship);
  draw_field();
}




// Hauptfunktion
void loop()
{

  elapsedMicros waiting;    // Auto updating

#undef HALT
#ifdef HALT
  // HALTing Game (Debug&Screenshot)
  if (!digitalRead(BUTT) == HIGH)
  {
#endif

    rx_points = 0;
    char buf[12];

    video();
    // Punktezähler ausgeben
    draw_string("Points:", 100, 150, 6);
    draw_string(itoa(score, buf, 10), 800, 150, 6);

    // FPS Todo: Debug Switch?!
    draw_string("FPS:", 3000, 150, 6);
    draw_string(itoa(fps, buf, 10), 3400, 150, 6);

    num_points = rx_points;

#ifdef HALT
  }
#endif

  /*
    // if there are any DMAs currently in transit, wait for them
    // to complete.
    while (!spi_dma_tx_complete())
      ;

    // now start any last buffered ones and wait for those
    // to complete.
    spi_dma_tx();
    while (!spi_dma_tx_complete())
      ;

    // flag that we have started an output frame
    //digitalWriteFast(DEBUG_PIN, 1);

    // force a reference voltage write on every cycle
    spi_dma_cs = SPI_DMA_CS_BEAM_OFF;
    mpc4921_write(1, 2048);
    spi_dma_cs = SPI_DMA_CS_BEAM_ON;
  */

  for (unsigned n = 0 ; n < num_points ; n++)
  {
    const uint32_t pt = points[n];
    uint16_t x = (pt >> 12) & 0xFFF;
    uint16_t y = (pt >>  0) & 0xFFF;
    unsigned intensity = (pt  >> 24) & 0x3F;

#ifndef FULL_SCALE
    x = (x >> 1) + 1024;
    y = (y >> 1) + 1024;
#endif

    if (intensity == 0)
      draw_moveto(x, y);
    else
      draw_lineto(x, y, intensity);
  }

  // go to the center of the screen, turn the beam off
  brightness(0);
  goto_x(REST_X);
  goto_y(REST_Y);

  // the USB loop above will flush eventually
  // digitalWriteFast(DEBUG_PIN, 0);

  //while (waiting < 10000)   //limit frame rate 100fps max
  // ;
  fps = 1000000 / waiting;

}



