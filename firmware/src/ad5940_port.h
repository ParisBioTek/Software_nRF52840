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

/* Allumer/éteindre l'alimentation du capteur via le load switch (P1.00). */
void  ad5940_power_on(void);
void  ad5940_power_off(void);

/* Initialiser l'AD5940 après power-on :
 *   reset matériel, horloge, FIFO, séquenceur, GPIO AD5940, mesure LFOSC.
 * Retourne la fréquence LFOSC mesurée (Hz), ou une valeur < 0 si l'AD5940 ne
 * répond pas sur le SPI (ADIID != 0x4144) — la mesure doit alors être annulée. */
float ad5940_platform_init(void);

/* Lit l'identifiant ADI de l'AD5940 (registre ADIID). Doit valoir 0x4144 si le
 * SPI/câblage est correct. Nécessite un AD5940_Initialize() préalable. */
uint32_t ad5940_read_adiid(void);

#endif /* AD5940_PORT_H */
