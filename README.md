# Cablebot

Contrôle du robot parallèle à câbles dans le Hall 3 de l'ENSAM Paris.<br>
*NB : sur VisualStudio Code, appuyer sur Ctrl+Shift+V pour afficher le .md*

---

## Prérequis et installation

Avant toute chose, assurez-vous d’avoir installé toutes les librairies Arduino nécessaires via le **Library Manager** :

* **TimerOne**
* **TMCStepper**
* **CircularBuffer**
* **Simple Web Serial**

Si vous souhaitez piloter le robot via l’interface web :

* Installez l’extension **Live Server** dans Visual Studio Code (ou tout autre serveur local capable de servir des fichiers HTML).

---

## Configuration matérielle

* Branchez la carte Arduino correctement, en vous assurant que le moteur, les capteurs, et les autres composants sont connectés selon le schéma du projet.
* Configurez la communication série à un **baud rate de 250000** dans le code Arduino et dans votre terminal ou interface web.

---

## Compilation et déploiement du code Arduino

1. Dans le fichier principal Arduino (`.ino`), assurez-vous d’activer la communication Web Serial en procédant comme suit :

   * Décommentez la ligne d’inclusion :

     ```cpp
     // #include "WebSerial.h"
     ```
   * Décommentez l’appel à l’initialisation dans `setup()` :

     ```cpp
     // web.setup();
     ```
   * Décommentez l’appel à la mise à jour dans la boucle `loop()` :

     ```cpp
     // web.update();
     ```
   * Commentez les lignes d’affichage directe des positions sur le port série (pour éviter les conflits) :

     ```cpp
     steppers.getCurrentPosition().print(); Serial.println();
     ```
   * Décommentez la ligne d’affichage via Web Serial :

     ```cpp
     // web.log("POS\t"+steppers.getCurrentPosition().to_String());
     ```

2. Compilez et téléversez le code sur la carte Arduino.

---

## Utilisation de l’interface web

1. Ouvrez le dossier contenant les fichiers de l’interface (`CableRobotControl.html`, `styles`, `scripts`, etc.).
2. Lancez un serveur local avec **Live Server** ou tout autre outil équivalent (par exemple : clic droit sur `CableRobotControl.html` → *Open with Live Server* ou cliquez sur l'icone *live server* en bas à droite de la fenêtre).
3. Si ce n'est pas fait automatiquement, ouvrez la page dans un navigateur compatible.
4. Connectez-vous à la carte Arduino via Web Serial (un bouton ou invite devrait apparaître dans l’interface — si ce n’est pas le cas, vérifiez que la bibliothèque `simple-web-serial.min.js` est bien chargée).
5. Vous pouvez maintenant contrôler le robot avec l’interface :

   * Les contrôles 3D (via **three.js**) permettent d’orbiter autour de la scène.
   * Utilisez le clavier :

     * **r** : basculer en mode rotation de l’effecteur
     * **t** : basculer en mode translation
   * Envoyez des commandes de déplacement ou actions spécifiques via l’interface Web Serial.

---

## Fonctionnalités du programme

* Contrôle en temps réel des moteurs pas à pas du robot grâce aux commandes JSON reçues par Web Serial.
* Interface graphique 3D interactive basée sur **three.js**.
* Gestion asynchrone des mouvements avec tampon (buffer) dans le contrôleur Arduino.
* Logs et feedbacks des positions moteurs directement sur l’interface Web.

---

## Conseils et dépannage

* Assurez-vous que le baud rate défini dans l’interface et dans l’Arduino correspond exactement.
* Si l’interface Web ne détecte pas le port série, vérifiez que le navigateur a bien les permissions d’accès au port série Web USB.
* En cas de problème de communication, relancez l’Arduino et rechargez la page web.
* Pour modifier la vitesse ou les positions cibles, adaptez les paramètres dans les commandes JSON envoyées (voir le fichier `robot_control.js` pour les exemples).

---

N’hésitez pas à consulter les fichiers source pour mieux comprendre la logique de communication entre la page web et le robot.