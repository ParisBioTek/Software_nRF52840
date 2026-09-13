#ifndef AD5940_PORT_H
#define AD5940_PORT_H

#include <stdint.h>

/* ── Fonctions requises par la librairie ADI ──────────────────────────────── */
void     AD5940_CsClr(void);
void     AD5940_CsSet(void);
void     AD5940_RstClr(void);
void     AD5940_RstSet(void);
void     AD5940_Delay10us(uint32_t time);
void     AD5940_ReadWriteNBytes(unsigned char *pSendBuffer,
                                unsigned char *pRecvBuff,
                                unsigned long  length);
uint32_t AD5940_GetMCUIntFlag(void);
uint32_t AD5940_ClrMCUIntFlag(void);  /* déclaré aussi dans ad5940.h — doit retourner uint32_t */

/* ── API propre au port Zephyr ───────────────────────────────────────────── */

/* Configurer les GPIO (CS, RST, alimentation). À appeler une seule fois. */
void  ad5940_hw_init(void);

/* Allumer/éteindre l'alimentation du capteur via le load switch (P1.00).
 * power_on retourne 0, ou le premier errno GPIO rencontré. */
int   ad5940_power_on(void);
void  ad5940_power_off(void);

/* Initialiser l'AD5940 après power-on :
 *   reset matériel, horloge, FIFO, séquenceur, GPIO AD5940, mesure LFOSC.
 * Retourne la fréquence LFOSC mesurée (Hz), ou une valeur < 0 si l'AD5940 ne
 * répond pas sur le SPI (ADIID != 0x4144) — la mesure doit alors être annulée. */
float ad5940_platform_init(void);

/* Lit l'identifiant ADI de l'AD5940 (registre ADIID). Doit valoir 0x4144 si le
 * SPI/câblage est correct. Nécessite un AD5940_Initialize() préalable. */
uint32_t ad5940_read_adiid(void);

/* Diagnostics conservés en RAM pour être remontés au PC via STATUS. Les
 * compteurs sont remis à zéro au début de chaque ad5940_platform_init(). */
void     ad5940_diag_reset(void);
uint32_t ad5940_spi_error_count(void);
int32_t  ad5940_spi_last_error(void);
int32_t  ad5940_platform_last_error(void);
uint32_t ad5940_platform_adiid(void);
uint32_t ad5940_platform_chipid(void);
int32_t  ad5940_spi_selected_mode(void);
/* Deux ADIID 16 bits empaquetés par mot : mode 0 dans les bits 15:0,
 * mode 1 dans 31:16 ; même convention pour les modes 2/3. 0xFFFF=non essayé. */
uint32_t ad5940_spi_mode_ids01(void);
uint32_t ad5940_spi_mode_ids23(void);
/** Bits 0..3 : modes SPI dont l'ADIID a réellement été lu. */
uint32_t ad5940_spi_mode_tried_mask(void);
int32_t  ad5940_gpio_last_error(void);
/* État mémorisé juste avant la première sonde SPI : bit0=PWR commandé actif,
 * bit1=CS commandé actif (bas), bit2=RESET commandé actif ; bits 8/9/10 =
 * latches physiques HIGH de PWR/CS/RESET. */
uint32_t ad5940_platform_gpio_state(void);
int32_t  ad5940_lfosc_cal_error(void);
uint32_t ad5940_lfosc_measured_hz(void);
uint32_t ad5940_lfosc_used_hz(void);
uint32_t ad5940_lfosc_fallback_used(void);

#endif /* AD5940_PORT_H */
