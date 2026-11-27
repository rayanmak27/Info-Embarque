# Temp_Moteur : température du moteur

Ali. Thermocouple de type K sur un module MAX6675 ou MAX31855, lu en SPI par la Pi, une fois par seconde au créneau 8 (800 ms).

## Câblage

| Module | Raspberry Pi (4 ou 5) | Broche |
|---|---|---|
| VCC | 3,3 V | 1 (ou 17) |
| GND | masse | 25 (ou 6, 9, 14, 20, 30, 34, 39) |
| CLK | GPIO11, SCLK de SPI0 | 23 |
| DO | GPIO9, MISO de SPI0 | 21 |
| CS | GPIO8, CE0 de SPI0 | 24 |
| non relié | GPIO10, MOSI de SPI0 | 19, réservé |

- **Jamais en 5 V** : DO suit l'alimentation et une broche de la Pi ne supporte pas 5 V ; un MAX31855 meurt au-delà de 4 V.
- Bornier de la sonde : fil jaune sur Yellow, rouge sur Red. Si la valeur baisse quand on chauffe la pointe, les fils sont inversés.
- Sur une voiture : sonde à jonction isolée, module loin de la chaleur (MAX6675 : 85 °C au plus).

## Préparer la Pi

Dans `/boot/firmware/config.txt` : `dtparam=spi=on`, puis redémarrer. `ls /dev/spidev0.*` doit montrer `spidev0.0`. Même réglage sur une Pi 4 et une Pi 5.

## Essais sur la Pi

Depuis `Temp_Moteur` :

```
gcc -Wall -Wextra -o essai/puce essai/identifier_puce.c && ./essai/puce
gcc -Wall -Wextra -o essai/essai essai/essai_tempmoteur.c tempmoteur.c && ./essai/essai
```

1. `essai/puce` dit quelle puce porte le module (pointe à l'air libre) et affiche les trames brutes.
2. Si c'est un MAX31855, ajouter `-DTEMPMOTEUR_PUCE=31855` à toutes les compilations (le défaut est 6675).
3. `essai/essai` : 60 mesures, une par seconde, avec la durée de chaque lecture.
   - pièce : environ 20 °C (10 °C : MAX31855 lu comme un MAX6675) ;
   - pointe dans la main : la valeur monte ;
   - sonde débranchée : « sonde ouverte ».

## Dans main.c

```c
#include <math.h>
#include "Temp_Moteur/tempmoteur.h"

tempmoteur_init();                       // une fois, au démarrage
...
case 8: // 800 ms
    if (tempmoteur_read(&d.temp_moteur) < 0)
        d.temp_moteur = NAN;             // le journal écrit nan, personne n'invente de valeur
    break;
...
tempmoteur_close();
```

Aucun appel au créneau 0 : la puce relance seule sa conversion à la fin de chaque lecture. La lecture ne bloque pas : un seul transfert SPI, 16 ou 32 coups d'horloge à 1 MHz, quelques dizaines de µs en tout.

| Retour | Valeur | Sens |
|---|---|---|
| 0 | | température en °C dans `*temperature` |
| `TEMPMOTEUR_ERR_SPI` | `-ENODEV` | SPI absent ou `tempmoteur_init` pas appelé |
| `TEMPMOTEUR_ERR_SONDE` | `-EIO` | thermocouple ouvert ou en court-circuit |
| `TEMPMOTEUR_ERR_TRAME` | `-EPROTO` | module absent ou mauvaise puce choisie |
| `TEMPMOTEUR_ERR_ATTENTE` | `-EAGAIN` | première conversion pas finie, juste après l'init |

Mêmes codes `-errno` que `Temp_Ambiante` : `-EAGAIN` veut dire « pas encore prêt » pour les deux températures.

## Sans le module : simulation

`simulation/tempmoteur_simu.c` a la même interface. Le compiler **à la place de** `tempmoteur.c` pour essayer la boucle, l'écran et le journal sans thermocouple : la température part de 20 °C et monte vers 90 °C (thermostat), constante de temps de 120 s au ralenti (réglable : `-DSIMU_TAU_S=600`).

Pour qu'elle suive le moteur, appeler `tempmoteur_simu_regime(d.rpm)` (`simulation/tempmoteur_simu.h`) après la mesure du régime : elle chauffe d'autant plus vite que le régime monte (4 fois plus vite à 3200 tr/min qu'à 800) et refroidit vers 20 °C quand le régime vaut 0. Sans cet appel, le moteur est supposé au ralenti.

Pour la démo, la vraie sonde et une vraie source de chaleur valent mieux : main (environ 33 °C), eau glacée (0 °C), eau bouillante (environ 100 °C), sèche-cheveux ou décapeur.

## Essai d'intégration : la boucle du projet

`essai/boucle_simu.c` reproduit l'exécutif du rapport commun : 10 créneaux de 100 ms, réveil à date absolue, régime au créneau 0, T moteur au créneau 8, une ligne de journal (CSV) au créneau 9. Il peut servir de point de départ pour `main.c`.

```
gcc -Wall -Wextra -DAVEC_SIMU -DSIMU_TAU_S=12 -o essai/boucle essai/boucle_simu.c simulation/tempmoteur_simu.c && ./essai/boucle > journal.csv
gcc -Wall -Wextra -o essai/boucle-reel essai/boucle_simu.c tempmoteur.c && ./essai/boucle-reel 60
```

Lancé sur la Pi le 27/11 : 150 lectures simulées sans erreur, réveils à 0,2 ms près ; sans SPI activé, `tempmoteur_init` et chaque lecture rendent -19 (`-ENODEV`), le journal écrit `nan` et la boucle continue.

## Tests sur PC, sans Pi

Depuis `Temp_Moteur/tests` (Linux ou Mac) :

```
cc -Ifake -I.. -DTEMPMOTEUR_PUCE=6675 test_tempmoteur.c -lm && ./a.out
cc -Ifake -I.. -DTEMPMOTEUR_PUCE=31855 test_tempmoteur.c -lm && ./a.out
cc -Ifake test_puce.c && ./a.out
cc test_simu.c && ./a.out
```

Le SPI est remplacé par un faux périphérique qui renvoie des trames choisies (exemples des fiches, défauts, trames fausses, lectures trop rapprochées).

## Limites

- Un MAX6675 ne mesure pas sous 0 °C : à 0 °C, sa trame vaut 0 et passe pour un module absent.
- Lire un MAX6675 arrête sa conversion : le module ne relit pas la puce moins de 250 ms après la lecture précédente et rend la dernière valeur.
