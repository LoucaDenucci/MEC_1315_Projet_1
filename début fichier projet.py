# -*- coding: utf-8 -*-
"""
Created on Sun Oct  8 20:11:38 2023

@author: noahd
"""

import numpy as np
import copy
from MEC1315_STL import *
from mes_fonctions_complets import *

# Créer listes
planete=list(LireSTL('planete.stl'))
minion=list(LireSTL('Minion2.stl'))
cylindre=list(LireSTL('cylindre.stl'))
triangle=list(LireSTL('triangle.stl'))
mini_planete=list(LireSTL('mini_planete.stl'))
cube=list(LireSTL('cube.stl'))

# Planete principal
central=homothetie(planete, 2/3)

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
satellite = rotation(satellite,np.pi/8,[0,1,0])
satellites = rep_circulaire(satellite, 3, np.array([400, 300, 0]), 1)

# Galaxy 
galaxy = rep_perso(8, planete_colonisee, planete, 2/3) #Répétition selon la suite de fibonacci
spirale = spirale_perso(mini_planete, 8, 56)

# Étoiles

triangle1=list(LireSTL('triangle.stl'))
triangle2= rotation(triangle1, np.pi, [0,1,0])
triangle2 = translation(triangle2, [0, 0, 1])

triangle1_2 = fusion([triangle1, triangle2])
triangle1_2 = affinite_vectorielle(triangle1_2, 1, 1.25, 0.25)

triangle3 = rotation(triangle1_2, np.pi, [1,0,0])
triangle3 = translation(triangle3, [0,0.75,0])
triangle3 = translation(triangle3, [0,0,0.25])

étoile = fusion([triangle1_2, triangle3])
étoile = homothetie(étoile, 50)
étoile = rotation(étoile, np.pi/2, [1,0,0])

étoiles = étoiles_partout(étoile)

# Fusée avec minion

    # création pointe de la fusée

minion=list(LireSTL('Minion2.stl'))
cylindre=list(LireSTL('cylindre.stl'))
triangle=list(LireSTL('triangle.stl'))

cone = translation(triangle, [-1,0,0])
cone = rotation(cone, np.pi/2, [0,1,0])
cone = rotation(cone, -np.pi/2/2, [1,0,0])
cone = translation(cone, [-0.5,0,0])
cone = translation(cone, [0,-(2**(1/2)/2),0])
reacteur = copy.deepcopy(cone) # cette copy sera utilisée lors de la création des réacteurs
cone = affinite_vectorielle(cone, 0.001, 1, 1)
cone = rep_circulaire(cone, 200, [0,0,0], 1)
cone = homothetie(cone, 1.25)


    # création corp de la fusée

cylindre = translation(cylindre, [0,0,-1])
cylindre = rep_circulaire(cylindre, 100, [0,0,0], 1)
cylindre = affinite_vectorielle(cylindre, 1, 1, 3)


    # création des réacteurs

reacteur = homothetie(reacteur, 2)
reacteur = translation(reacteur, [0,0,-3.5])
reacteur = affinite_vectorielle(reacteur, 0.1, 1, 1)
reacteur = rep_circulaire(reacteur, 3, [0,0,0], 1)

    # ajout de minion sur fusée

minion = homothetie(minion, 1/30)
minion = centrer(minion)
minion = rotation(minion, np.pi/2, [0,1,0])
minion = rotation(minion, -np.pi/2, [1,0,0])
minion = translation(minion, [0,0,-1])

# création de l'objet fusee

fusee = fusion([cone, cylindre, reacteur, minion])
fusee = homothetie(fusee, 20)
fusee = rotation(fusee, -np.pi/2, [0,1,0])
fusee = translation(fusee, [0,0,500])

# Fusion et export du fichier STL
scene = fusion([central, anneau_p, anneau_s1, anneau_s2, galaxy, satellites, spirale, étoiles, fusee])
f,v,n = scene[0], scene[1], scene[2]
EcrireSTLASCII('Scene_25.stl', f, v, n)
