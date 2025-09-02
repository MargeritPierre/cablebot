# -*- coding: utf-8 -*-
"""
Created on Mon Mar 17 14:08:22 2025

@author: Ngatam
"""
################################# Import ######################################

import sympy
import scipy
import numpy as np

from numpy import linalg
from sympy import sin, cos, sqrt, Matrix
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
lambda_1 = sqrt((X_e - X[1] + a[1]*cos(phi_1) - b[1]*sin(phi_1))**2 + (Y_e - Y[1] + a[1]*sin(phi_1) + b[1]*sin(phi_1))**2)
lambda_2 = sqrt((X_e - X[2] + a[2]*cos(phi_1) - b[2]*sin(phi_1))**2 + (Y_e - Y[2] + a[2]*sin(phi_1) + b[2]*sin(phi_1))**2)
lambda_3 = sqrt((X_e - X[3] + a[3]*cos(phi_1) - b[3]*sin(phi_1))**2 + (Y_e - Y[3] + a[3]*sin(phi_1) + b[3]*sin(phi_1))**2)
lambda_4 = sqrt((X_e - X[4] + a[4]*cos(phi_1) - b[4]*sin(phi_1))**2 + (Y_e - Y[4] + a[4]*sin(phi_1) + b[4]*sin(phi_1))**2)
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
Xe = (lambda_3**2 - lambda_2**2 - l_1**2 + l_1*l) / (2*(l - l_1))
Ye = (lambda_1**2 - lambda_2**2 - h_1**2 + h_2**2 - L*(h_1 + h_2)) / (2*(h_2 - h_1 - L)) 
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
X = [0, 0, l_1, l_1] # position sur l'axe x_base des poulies
Y = [h_1, h_2, h_2, h_1] # position sur l'axe y_base des poulies 
a = [-l/2, -l/2, l/2, l/2] # position sur l'axe x_effecteur des points d'accroche
b = [-L/2, L/2, L/2, -L/2] # position sur l'axe y_effecteur des points d'accroche


## Modèle inverse - variables - position de l'effecteur
X_e = sympy.symbols("X_e")
Y_e = sympy.symbols("Y_e")
phi_1 = sympy.symbols("phi_1")


"""
########## Zone de test ##########
position initial:
    X = 112cm
    Y = 54cm
    
 angles initiaux:
     Nombres de pas des moteurs {'+' : dérouler, '-': enrouler} :
     Moteur  1  : -36.771
     Moteur  2  : -36.229
     Moteur  3  : -77.048
     Moteur  4  : -80.134
     

    """

## Modèle inverse - équations modèle analytique
lambda_1 = sqrt((X_e - X[0] + a[0]*cos(phi_1) - b[0]*sin(phi_1))**2 + (Y_e - Y[0] + a[0]*sin(phi_1) + b[0]*sin(phi_1))**2) # longueur du câble de la poulie 1 (en mm)
lambda_2 = sqrt((X_e - X[1] + a[1]*cos(phi_1) - b[1]*sin(phi_1))**2 + (Y_e - Y[1] + a[1]*sin(phi_1) + b[1]*sin(phi_1))**2) # longueur du câble de la poulie 2 (en mm)
lambda_3 = sqrt((X_e - X[2] + a[2]*cos(phi_1) - b[2]*sin(phi_1))**2 + (Y_e - Y[2] + a[2]*sin(phi_1) + b[2]*sin(phi_1))**2) # longueur du câble de la poulie 3 (en mm)
lambda_4 = sqrt((X_e - X[3] + a[3]*cos(phi_1) - b[3]*sin(phi_1))**2 + (Y_e - Y[3] + a[3]*sin(phi_1) + b[3]*sin(phi_1))**2) # longueur du câble de la poulie 4 (en mm)
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
#- si pas bon, changer 360/2*np.pi en 100/np.pi  (passage de degré en radian) -#


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


## Modèle inverse - Calcul de la Jacobienne du modèle cinématique inverse et test
#- faire un dico avec 'longueur' <-> lambda_i, 'pas' <-> p_i, 'angles' <-> q_i, dico que l'on pourra utiliser lors de l'appel de la fonction + mettre affichage en lien avec le dico

def inverse_test(X_e_exp, Y_e_exp, phi_1_exp):
    X_entrées = Matrix([X_e, Y_e, phi_1]) # Entrées
    Y_sorties = Matrix([lambda_1, lambda_2, lambda_3, lambda_4]) # Sorties
    J = Y_sorties.jacobian(X_entrées) # Jacobienne du modèle cinématique inverse 
    print("---- Modèle cinématique inverse pour test  ---- ")
    print("Variables d'entrée : \n", X)
    print("\nVariables de sortie : \n", Y)
    
    J = J.subs({X_e: X_e_exp, Y_e: Y_e_exp, phi_1: phi_1_exp})  # Substitution des valeurs d'entrées sur la Jacobienne
    X_entrées = X_entrées.subs({X_e: X_e_exp, Y_e: Y_e_exp, phi_1: phi_1_exp})  # Substitution des valeurs d'entrées sur le vecteur en entrée

    print("\nJacobienne  avec les valeurs expérimentales: \n", J) 
    pas_exp = np.dot(J, X_entrées) # calcul le produit matriciel de la Jacobienne et de la matrice des variables d'entrées
    print("\nNombres de pas des moteurs {'+' : dérouler, '-': enrouler} :")
    i, j = np.shape(pas_exp) # affichage des nombres de pas par moteur
    for k in  range(0, i):
        for l in range(0, j):
            print("Moteur ", k+1, " :", round(float(pas_exp[k][l]))) # arrondi des nombres de pas


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


