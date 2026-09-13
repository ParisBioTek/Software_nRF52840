#ifndef METHODS_H_
#define METHODS_H_

#include <stdbool.h>

/* Techniques d'analyse électrochimique sélectionnables depuis le PC (SMP START).
 * L'entier est transmis tel quel dans la clé "method" du payload CBOR. */
typedef enum {
    METHOD_CV  = 0,   /* voltampérométrie cyclique (RampTest.c / CV.c)          */
    METHOD_CA  = 1,   /* chronoampérométrie i–t (CA.c, logiciel-timé)           */
    METHOD_DPV = 2,   /* voltampérométrie à impulsions différentielles (DPV.c)  */
} analysis_method_t;

/* Contrat commun aux moteurs bloquants CA/DPV et aux sondes d'autorange CV.
 * Le callback ne doit effectuer aucun acces materiel : il consulte seulement
 * l'etat d'annulation protege par main.c. */
typedef bool (*measurement_stop_cb_t)(void);

/* Valeur de retour positive reservee a un arret demande par l'utilisateur.
 * Les erreurs techniques restent negatives et un succes complet vaut zero. */
#define MEAS_RUN_STOPPED 1

#endif /* METHODS_H_ */
