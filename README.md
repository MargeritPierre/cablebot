# cablebot
Control of the cable robot of the Hall 3 of ENSAM Paris


# Initialisation du code
Il faut avant tout installer les librairies manquantes disponibles au téléchargement dans l'onglet "Library Manager" de arduino.
> TimerOne
> TMCStepper
> CircularBuffer
> Simple Web Serial

Configurer l'arduino au baud rate 250000 

## Utilisation avec page web
1. décommenter `// #include "WebSerial.h"`
2. décommenter `//  web.setup();`
3. décommenter `//  web.update();`
4. passer de 1 à 0 dans `if (0 and Serial.available())`
5. commenter `steppers.getCurrentPosition().print(); Serial.println();`
6. décommenter `web.log("POS\t"+steppers.getCurrentPosition().to_String());`
