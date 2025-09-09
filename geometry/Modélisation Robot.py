# -*- coding: utf-8 -*-
"""
Created on Mon Mar 17 14:08:22 2025

@author: Ngatam
"""
################################# Import ######################################

import sympy
import scipy

import numpy as np
import matplotlib.pyplot as plt

from numpy import linalg
from sympy import sin, cos, sqrt, Matrix
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from sympy.abc import alpha, beta, phi, delta, theta, psi, omega



################################# Texte #######################################

"""
---- Paramètres du système ----
h_1 = 400 : hauteur poulie 1 (en mm)
h_2 = 2180 : hauteur poulie 2 (en mm)
L = 230 : longeur de la plaque de l'effecteur (en mm)
l = 230 : largeur de la plaque de l'effecteur (en mm)
l_1 = 2150 : distance entre 2 pieds de la structure (en mm)
K = 1.5 : rapport de transmission du ième enrouleur
e = 30 : rayon de l'enroleur i (en mm) 
rho = 0.005 : pas de l'enrouleur i (en mm)

---- Coefficients pour le calcul ----
X : position sur l'axe x_base des poulies 
Y : position sur l'axe y_base des poulies 
a : position sur l'axe x_effecteur des points d'accroche
b : position sur l'axe y_effecteur des points d'accroche
---
r = K * (e**2 + (rho**2)/2*np.pi) : coefficient d'enroulement des enrouleurs (supposé identiques car même enrouleurs)
lambda_1 = r*q_1 : lambda 1, longeur du câble 1
lambda_2 = r*q_2 : lambda 2, longeur du câble 2
lambda_3 = r*q_3 : lambda 3, longeur du câble 3
lambda_4 = r*q_4 : lambda 4,# longeur du câble 4


---- Variables ----
lambda_1 : longueur du câble de la poulie 1
lambda_2 : longueur du câble de la poulie 2
lambda_3 : longueur du câble de la poulie 3
lambda_4 : longueur du câble de la poulie 4
---
q_1 : position angulaire du moteur 1
q_2 : position angulaire du moteur 2
q_3 : position angulaire du moteur 3
q_4 : position angulaire du moteur 4
---
p_1 : nombre du pas sur le moteur 1
p_2 : nombre du pas sur le moteur 2
p_3 : nombre du pas sur le moteur 3
p_4 : nombre du pas sur le moteur 4
---
Xe: position de l'effecteur sur l'axe X de la structure
Ye: position de l'effecteur sur l'axe Y de la structure
phi_1 : position angulaire de l'effecteur par rapport au repère de la base


---- Equations ----
lambda_1 = - sqrt((X_e - X[0] + a[0]*cos(phi_1) - b[0]*sin(phi_1))**2 + (Y_e - Y[0] + a[0]*sin(phi_1) + b[0]*cos(phi_1))**2)
lambda_2 =   sqrt((X_e - X[1] + a[1]*cos(phi_1) - b[1]*sin(phi_1))**2 + (Y_e - Y[1] + a[1]*sin(phi_1) + b[1]*cos(phi_1))**2)
lambda_3 = - sqrt((X_e - X[2] + a[2]*cos(phi_1) - b[2]*sin(phi_1))**2 + (Y_e - Y[2] + a[2]*sin(phi_1) + b[2]*cos(phi_1))**2)
lambda_4 =   sqrt((X_e - X[3] + a[3]*cos(phi_1) - b[3]*sin(phi_1))**2 + (Y_e - Y[3] + a[3]*sin(phi_1) + b[3]*cos(phi_1))**2)
---
q_1 = lambda_1 / r
q_2 = lambda_2 / r
q_3 = lambda_3 / r
q_4 = lambda_4 / r
---
pas_1 = (360*q_1) / 2*np.pi
pas_2 = (360*q_2) / 2*np.pi
pas_3 = (360*q_3) / 2*np.pi
pas_4 = (360*q_4) / 2*np.pi
---
Poulie 1 et 3 : {'-' : Dérouler, '+': Enrouler} 
Poulie 2 et 4 : {'-': Enrouler, '+': Dérouler}

Par convention, le sens trigonométrique est noté '+', 
On va donc considérer qu'il s'agit de l'action de dérouler du câble
On va donc avoir sur toutes les poulies:
    '-' -> Enrouler du câble sur la poulie
    '-' -> sens trigonométrique des poulies 1 et 4, sens horaire des poulies 2 et 3
    
    '+' -> Dérouler du câbles sur la poulie
    '+' -> sens trigonométrique des poulies 2 et 3, sens horaire des poulies 1 et 4
    
    Pour avoir cette logique sur tout les poulies, on doit mettre des '-' devant les calculs de lambda_1 et lambda_4

"""



################################# Définition ##################################

## Paramètres du système
h_1 = 400 # (en mm) hauteur poulie 1
h_2 = 2180 # (en mm) hauteur poulie 2
l = 230 # (en mm) largeur de la plaque de l'effecteur
L = 230 # (en mm) longeur de la plaque de l'effecteur
l_1 = 2150 # (en mm) distance entre 2 pieds de la structure
K = 0.5 # rapport de transmission de l'enrouleur
e = 30 # (en mm) rayon de l'enroleur 
rho = 5 # (en mm) pas de l'enrouleur
pas_mot = 1.8 # (en °) pas du moteur => 200 pas pour 1 tour


## Coefficients pour le calcul
r = K * (e**2 + (rho**2)/2*np.pi) # coefficient d'enroulement des enrouleurs
X = [l_1, l_1, 0, 0] # position sur l'axe x_base des poulies
Y = [h_1, h_2, h_1, h_2] # position sur l'axe y_base des poulies 
a = [l/2, l/2, -l/2, -l/2] # position sur l'axe x_effecteur des points d'accroche
b = [-L/2, L/2, -L/2, L/2] # position sur l'axe y_effecteur des points d'accroche


## Modèle inverse - variables - position de l'effecteur
X_e = sympy.symbols("X_e")
Y_e = sympy.symbols("Y_e")
phi_1 = sympy.symbols("phi_1")

# Position intiale de l'effecteur - au centre du repère
X_0 = 1075  # Position initiale du centre de la plaque en X
Y_0 = 1075  # Position initiale du centre de la plaque en Y 


## Modèle inverse - équations modèle analytique
lambda_1 =   sqrt((X_0 + X_e - X[0] + a[0]*cos(phi_1) - b[0]*sin(phi_1))**2 + (Y_0 + Y_e - Y[0] + a[0]*sin(phi_1) + b[0]*cos(phi_1))**2) # longueur du câble de la poulie 1 (en mm)
lambda_2 =   sqrt((X_0 + X_e - X[1] + a[1]*cos(phi_1) - b[1]*sin(phi_1))**2 + (Y_0 + Y_e - Y[1] + a[1]*sin(phi_1) + b[1]*cos(phi_1))**2) # longueur du câble de la poulie 2 (en mm)
lambda_3 =   sqrt((X_0 + X_e - X[2] + a[2]*cos(phi_1) - b[2]*sin(phi_1))**2 + (Y_0 + Y_e - Y[2] + a[2]*sin(phi_1) + b[2]*cos(phi_1))**2) # longueur du câble de la poulie 3 (en mm)
lambda_4 =   sqrt((X_0 + X_e - X[3] + a[3]*cos(phi_1) - b[3]*sin(phi_1))**2 + (Y_0 + Y_e - Y[3] + a[3]*sin(phi_1) + b[3]*cos(phi_1))**2) # longueur du câble de la poulie 4 (en mm)
# ---
q_1 = lambda_1 / r # angle de rotation du moteur 1 (en °)
q_2 = lambda_2 / r # angle de rotation du moteur 2 (en °)
q_3 = lambda_3 / r # angle de rotation du moteur 3 (en °)
q_4 = lambda_4 / r # angle de rotation du moteur 4 (en °)
# ---
p_1 = (200*q_1) / 2*np.pi # nombre de pas sur le moteur 1
p_2 = (200*q_2) / 2*np.pi # nombre de pas sur le moteur 2
p_3 = (200*q_3) / 2*np.pi # nombre de pas sur le moteur 3
p_4 = (200*q_4) / 2*np.pi # nombre de pas sur le moteur 4





################################# Calcul ######################################

## Modèle inverse - Calcul de la Jacobienne du modèle cinématique inverse
def inverse():
    X = Matrix([X_e, Y_e, phi_1]) # Entrées
    Y = Matrix([p_1, p_2, p_3, p_4]) # Sorties
    J = Y.jacobian(X) # Jacobienne du modèle cinématique inverse 
    print("---- Modèle cinématique inverse ---- ")
    print("Variables d'entrée : \n", X)
    print("\nVariables de sortie : \n", Y)
    print("\nJacobienne : \n", J)
    return J


## Modèle inverse - Calcul de la Jacobienne du modèle cinématique inverse et test
#- faire un dico avec 'longueur' <-> lambda_i, 'pas' <-> p_i, 'angles' <-> q_i, dico que l'on pourra utiliser lors de l'appel de la fonction + mettre affichage en lien avec le dico

def inverse_test(X_e_exp, Y_e_exp, phi_1_exp, temps):
    X_entrées = Matrix([X_e, Y_e, phi_1]) # Entrées
    Y_sorties = Matrix([lambda_1, lambda_2, lambda_3, lambda_4]) # Sorties
    J = Y_sorties.jacobian(X_entrées) # Jacobienne du modèle cinématique inverse 
    print("\n---- Modèle cinématique inverse pour test  ---- ")
    #-  print("Variables d'entrée : \n", X)
    #- print("\nVariables de sortie : \n", Y)
    #- print("\nJacobienne : \n", J)

    J = J.subs({X_e: X_e_exp, Y_e: Y_e_exp, phi_1: phi_1_exp})  # Substitution des valeurs d'entrées sur la Jacobienne
    
    # Création du vecteur des vitesses des l'effecteur, sur l'axe x, sur l'axe y et de rotation autour de l'axe z
    V_X_entrées = X_entrées.subs({X_e: X_e_exp/temps, Y_e: Y_e_exp/temps, phi_1: phi_1_exp/temps})  

    #- print("\nJacobienne  avec les valeurs expérimentales: \n", J) 
    V_lambda_exp = np.dot(J, V_X_entrées) # calcul le produit matriciel de la Jacobienne et de la matrice des variables d'entrées
    D_lambda_exp = V_lambda_exp*temps
    
    # Ajout de la longueur de base pour l'effecteur au centre
    longueur_initial = 930 
    for i in range(0, 4):     
        D_lambda_exp[i][0] = D_lambda_exp[i][0] + longueur_initial
    
    print("Pour un déplacement de: (", X_e_exp, "," , Y_e_exp, "), par rapport au point central")
    print("\nVitesse de déroulage/enroulage de câble par les moteurs (en mm)")
    print("{'+' : dérouler du câble, '-': enrouler du câble} :")
    i, j = np.shape(V_lambda_exp)
    for k in  range(0, i):
        print("\nCâble ", k+1, " :")
        print("Vitesse : ", round(float(V_lambda_exp[k][0]), 3), " mm/s") 
        print("Longeur du câble : ", round(float(V_lambda_exp[k][0]*temps), 3), "mm") 

    return V_lambda_exp, D_lambda_exp


def animation_test(longueur_1, longueur_2, longueur_3, longueur_4):
    print("\nlongueur 1 = ", longueur_1)
    print("\nlongueur 2 = ", longueur_2)
    print("\nlongueur 3 = ", longueur_3)
    print("\nlongueur 4 = ", longueur_4)

    cable_lengths = {
        "Câble 1": longueur_1,
        "Câble 2": longueur_2,
        "Câble 3": longueur_3,
        "Câble 4": longueur_4,
    }

    num_frames = len(longueur_1)
    plaque_size = 230

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(0, 2150)
    ax.set_ylim(0, 2150)
    ax.set_zlim(0, 500)
    ax.set_title("Déplacement de la plaque dans le plan XY")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    # Repère 3D
    ax.quiver(0, 0, 0, 100, 0, 0, color='r', arrow_length_ratio=0.05)
    ax.quiver(0, 0, 0, 0, 100, 0, color='g', arrow_length_ratio=0.05)
    ax.quiver(0, 0, 0, 0, 0, 100, color='b', arrow_length_ratio=0.05)

    anchor_points = {
        "Câble 1": [2150, 0, 0],
        "Câble 2": [2150, 2150, 0],
        "Câble 3": [0, 0, 0],
        "Câble 4": [0, 2150, 0],
    }

    lines = []
    texts = []
    plaque = None

    def update(frame):
        nonlocal lines, texts, plaque

        for line in lines:
            line.remove()
        for text in texts:
            text.remove()
        if plaque:
            plaque.remove()

        lines = []
        texts = []

        # Longueurs des câbles à cet instant
        l1 = cable_lengths["Câble 1"][frame]
        l2 = cable_lengths["Câble 2"][frame]
        l3 = cable_lengths["Câble 3"][frame]
        l4 = cable_lengths["Câble 4"][frame]
        
        print("\nl1 = ", l1)
        print("l2 = ", l2)
        print("l3 = ", l3)
        print("l4 = ", l4)
        

        # Calcul du centre de la plaque
        dx = (l2 + l4 - l1 - l3) * 0.25
        dy = (l1 + l2 - l3 - l4) * 0.25
        cx = 1075 + dx
        cy = 1075 + dy
        cz = 0

        half = plaque_size / 2
        p1 = [cx + half, cy - half, cz]
        p2 = [cx + half, cy + half, cz]
        p3 = [cx - half, cy - half, cz]
        p4 = [cx - half, cy + half, cz]

        for name, anchor, corner in zip(
            ["Câble 1", "Câble 2", "Câble 3", "Câble 4"],
            [anchor_points["Câble 1"], anchor_points["Câble 2"], anchor_points["Câble 3"], anchor_points["Câble 4"]],
            [p1, p2, p3, p4],
        ):
            line = ax.plot([anchor[0], corner[0]], [anchor[1], corner[1]], [anchor[2], corner[2]], 'gray')[0]
            lines.append(line)

            mid = [(anchor[i] + corner[i]) / 2 for i in range(3)]
            text = ax.text(mid[0], mid[1], mid[2] + 10, name, color='black', fontsize=9, ha='center')
            texts.append(text)

        # Matrices pour le plot_surface
        X = np.array([[p1[0], p2[0]], [p3[0], p4[0]]], dtype=float)
        Y = np.array([[p1[1], p2[1]], [p3[1], p4[1]]], dtype=float)
        Z = np.array([[cz, cz], [cz, cz]], dtype=float)

        plaque = ax.plot_surface(X, Y, Z, color='skyblue', alpha=0.8)

        return lines + [plaque] + texts
    
    ani = animation.FuncAnimation(fig, update, frames=num_frames, interval=600, blit=False)
    return ani


def inverse_plot(X_e_final, Y_e_final, phi_1_final, nombre_iteration, temps):
    X1 = np.linspace(0, X_e_final, nombre_iteration)
    X2 = np.linspace(0, Y_e_final, nombre_iteration)
    PHI = np.linspace(0, phi_1_final, nombre_iteration)
    Etape = np.arange(nombre_iteration)

    D1, D2, D3, D4 = [], [], [], []
    V1, V2, V3, V4 = [], [], [], []

    for i, j, k in zip(X1, X2, PHI):
        vitesse_cable_actuel, deplacement_cable_actuel = inverse_test(i, j, k, temps)
        print("\ndeplacement_cable_actuel: ", deplacement_cable_actuel)

        D1.append(deplacement_cable_actuel[0][0])
        D2.append(deplacement_cable_actuel[1][0])
        D3.append(deplacement_cable_actuel[2][0])
        D4.append(deplacement_cable_actuel[3][0])

        V1.append(vitesse_cable_actuel[0][0])
        V2.append(vitesse_cable_actuel[1][0])
        V3.append(vitesse_cable_actuel[2][0])
        V4.append(vitesse_cable_actuel[3][0])

    # Plot des déplacements
    fig_1, axs_1 = plt.subplots(nrows=2, ncols=2)
    fig_1.suptitle("Tailles des câbles")
    for ax, D, title in zip(axs_1.flat, [D1, D2, D3, D4], ["Câble 1", "Câble 2", "Câble 3", "Câble 4"]):
        ax.plot(Etape, D)
        ax.set_title(title)
        ax.set_xlabel("Itération")
        ax.set_ylabel("Déplacement [mm]")
        ax.grid()

    # Plot des vitesses
    fig_2, axs_2 = plt.subplots(nrows=2, ncols=2)
    fig_2.suptitle("Vitesses linéaires des câbles")
    for ax, V, title in zip(axs_2.flat, [V1, V2, V3, V4], ["Câble 1", "Câble 2", "Câble 3", "Câble 4"]):
        ax.plot(Etape, V)
        ax.set_title(title)
        ax.set_xlabel("Itération")
        ax.set_ylabel("Vitesse [mm/s]")
        ax.grid()

    # Animation
    global ani
    ani = animation_test(D1, D2, D3, D4)

    # Tracée de la trajectoire estimé
    CX = []
    CY = []
    
    for l1, l2, l3, l4 in zip(D1, D2, D3, D4):
        # Méthode approximative (même que dans animation_test, à affiner plus tard)
        dx = (l2 + l4 - l1 - l3) * 0.25
        dy = (l1 + l2 - l3 - l4) * 0.25
        cx = 1075 + dx
        cy = 1075 + dy
        CX.append(cx)
        CY.append(cy)
    
    # Plot position estimée
    fig_pos, ax_pos = plt.subplots()
    ax_pos.plot(CX, CY, label="Trajectoire estimée", marker='o')
    ax_pos.set_title("Position estimée du centre de la plaque")
    ax_pos.set_xlabel("X [mm]")
    ax_pos.set_ylabel("Y [mm]")
    ax_pos.grid()
    ax_pos.legend()

    # Affichage
    plt.show()

        

    

## Modèle direct - Calcul de la pseudo inverse de la Jacobienne
def direct():
    J = inverse()
    print("\n---- Modèle cinématique direct ---- ")
    print("Variables d'entrée : \n", Y)
    print("\nVariables de sortie : \n", X)
    
    # J = J.subs({X_e: 1, Y_e: 2, phi_1: 3})  # Substitution temporaire des valeurs variables
    
    J_T = np.transpose(J) 
    print("\nJ_T : \n", J_T)
    print("\nnp.shape(J_T) : \n", np.shape(J_T))
    
    J_prod = sympy.Matrix(np.dot(J_T, J))
    print("\nJ_prod : \n", J_prod)
    print("\nnp.shape(J_prod) : \n", np.shape(J_prod))
    print("\nJ_prod.det() : \n", J_prod.det())
    
    J_prod_inv = J_prod.inv()
    print("\nJ_prod_inv : \n", J_prod_inv)
    print("\nnp.shape(J_prod_inv) : \n", np.shape(J_prod_inv))
    
    J_pseudo_inv = np.dot(J_prod_inv, J_T)
    print("\nJ_pseudo_inv : \n", J_pseudo_inv)
    print("\nnp.shape(J_pseudo_inv) : \n", np.shape(J_pseudo_inv))


