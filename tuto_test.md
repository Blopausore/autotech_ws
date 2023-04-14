# Docu test de la voiture

Lancer la jetson avec le mot de passe : Intech2022

## Généralité

Pour bien faire : penser à changer le code sur vos pc puis à git pull sur la jetson
Sur la jetson il faut build le package avec ros :
'''
source ~/.bashrc
colcon build
'''
Donc essayer le moins possible de modifier du code sur la jetson. 
Le dossier monthlery est buggé sur le github mais pas sur la jetson, c'est normal sur le github si il n'est pas accessible.
## Lidar to ai

### Step 1
* Alimentation à moins de 24V (23.--V)
* brancher le lidar : brancher le câble noir (la masse) en premier puis le rouge
* brancher câble ethernet
* lancer la commande : 
'''
ros2 run urg_node urg_node_driver --ros-args --params-file ./install/urg_node/share/urg_node/launch/urg_node_ethernet.yaml
'''
* lancer le noeud lidar_to_ai : ros2 run monthlery lidar_to_ai
* lancer le noeud ai_node : ros2 run autotech_ros2_pkg ai_node
* observer les topics et voir si le retour du lidar est cohérent et voir si la réponse du modèle l'est aussi

### Step 2 : Vérifier le bon fonctionnement des fonctions de linerarisation

### Step 3 : Vérifier que les bonnes échelles sont employées

## SSH
### Server
Openssh-server a déjà était installé.
Lancer le server :
'''
sudo systemctl enable ssh
sudo systemctl start ssh
'''

Arreter le server, on ferme tout d'abord les connections existantes
'''
killall sshd
sudo systemctl stop ssh
'''

### Client
Pour trouver l'ip de la Jetson : ifconfig | grep inet 
Puis prendre celle qui semble vraisemblable ou bien ifconfig : donnera l'ensemble est regarder dans le paragraph wlps.
Pour se connecter : 
'''
ssh master@[ipv4]
'''
Puis donner le mot de passe.

## Arduino et fonctionnement de la commande vitesse et angle

Lancer teleop_node et com_node pour pouvoir tester la voiture.
Il faudra checker si la voiture suis bien les commandes demandés et essayer de noter les valeurs de la voiture qui sont intérressantes.

### Pin arduino employé
Ne pas hésiter à vérifier le bon branchement des pins
* PIN D09 : commande vitesse lineaire
* PIN D10 : commande vitesse angulaire

* PIN D2 | GND : GND (masse)
* PIN D3 | 5V  : 5V

### Check-up matériel

#### Batterie 
A tester avec l'ecran à 5 (peut etre plus) broches qui se trouve dans l'atelier.
Il suffit de brancher ces 5 branches a la batterie (pas l'alimentation general mais les cables collé)

Courant ok : < 12V

#### Moteur controlleur
Il possède un switch on/off (C'est bien de le rappeler).
Un bip = signal recu pas compris

#### Arduino 
Bouton reset dessus : a tout moment il suffisait d'appuyer







