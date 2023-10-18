# -*- coding: utf-8 -*-
"""
Created on Sun Oct  8 20:11:38 2023

@author: noahd
"""

import numpy as np
from MEC1315_STL import *
from mes_fonctions_complets import *

# Créer listes
planete=list(LireSTL('planete.stl'))
minion=list(LireSTL('Minion2.stl'))
cylindre=list(LireSTL('cylindre.stl'))
triangle=list(LireSTL('triangle.stl'))
mini_planete=list(LireSTL('mini_planete.stl'))
cube=list(LireSTL('cube.stl'))


# Anneaux
anneau_p = rep_circulaire(rotation(cylindre, np.pi/2,[1,0,0]), 150, np.array([135, 0, 0]), 7) # Répétition circulaire
anneau_s1 = rep_circulaire(mini_planete, 120, [120, 0, 10], 1/50)
anneau_s2 = rep_circulaire(mini_planete, 120, [120, 0, -10], 1/50)


# Drapeau
    # dimensions
cylindre = affinite_vectorielle(cylindre, 0.5, 0.5, 3)
triangle = affinite_vectorielle(triangle, 1, 6, 4)
triangle = homothetie(triangle, 0.15)
    
    # Placer le triangle en haut et à gauche du cylindre
deplacement_en_y = max(cylindre[1][:,1])
deplacement_en_z = max(cylindre[1][:,2])-(max(triangle[1][:,2])-min(triangle[1][:,2]))
deplacement_triangle = np.array([0, deplacement_en_y, deplacement_en_z])
triangle = translation(triangle, deplacement_triangle)
    
drapeau = fusion([cylindre, triangle])
drapeau = homothetie(drapeau, 20)
drapeau = rotation(drapeau, -np.pi/2, [0, 0, 1])

# Minion et drapeau
drapeau = centrer(drapeau)
drapeau = translation(drapeau, np.array([50, -2, -4])) # translation
minion = centrer(homothetie(minion, 1.25))
minion_drapeau= fusion([drapeau, minion])


# Planète colonisée
planete = translation(planete, np.array([0, 0, -(max(planete[1][:,2])-min(planete[1][:,2]))/2])) # translation vers z négatifs de la longueur du rayon
planete_colonisee = fusion([minion_drapeau, planete]) # fusion des deux objets et création du fichier "planete_colonisee.stl"


# Satellite
rectangles = repetition_rectiligne(affinite_vectorielle(cube, 0.5, 1.5, 0.2), 4, 1)

aile_sup = centrer(rectangles) # les 4 panneaux supérieurs de l'aile
aile_sup = translation(aile_sup, np.array([0, 0.9, 0]))
aile_mid = centrer(affinite_vectorielle(cube, 5, 0.4, 0.5)) # centre de l'aile
aile_mid = translation(aile_mid,np.array([0.5, 0, -0.2]))
aile_inf = translation(centrer(rectangles), np.array([0, -0.9, 0])) # les 4 panneaux inférieurs de l'aile

aile_1 = fusion([aile_sup, aile_mid, aile_inf]) # une des ailes du satellite
aile_2 = translation(rotation(aile_1, np.pi, [0, 0, 1]),np.array([7, 0, 0]))  # la deuxième aile du satellite
centre = centrer(affinite_vectorielle(cube, 2, 2.5, 2)) # Centre satellite
centre = translation(centre, np.array([3.5, 0, -0.8]))

satellite = homothetie(fusion([aile_1, aile_2, centre]), 10)
satellite = rotation(satellite,np.pi/4,[1,0,0])
satellite = translation(satellite, np.array([-300, 300, 0]))

#Galaxy 
galaxy=rep_perso(6, planete_colonisee, planete, 2/3) #Répétition selon la suite de fibonacci

# fusion et export du fichier STL
scene = fusion([anneau_p, anneau_s1, anneau_s2, galaxy, satellite])
f,v,n = scene[0], scene[1], scene[2]
EcrireSTLASCII('scene_final_XX.stl', f, v, n)
