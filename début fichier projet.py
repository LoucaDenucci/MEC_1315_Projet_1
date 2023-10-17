# -*- coding: utf-8 -*-
"""
Created on Sun Oct  8 20:11:38 2023

@author: noahd
"""


import numpy as np
from MEC1315_STL import *
from mes_fonctions_complets import *
#Créer  liste
planete=list(LireSTL('planete.stl'))
minion=list(LireSTL('Minion2.stl'))
cylindre=list(LireSTL('cylindre.stl'))
triangle=list(LireSTL('triangle.stl'))
mini_planete=list(LireSTL('mini_planete.stl'))
cube=list(LireSTL('cube.stl'))


#Anneau
Anneau_p=rep_circulaire(mini_planete, 120, [135, 0, 0], 1/50) #Répétition circulaire
Anneau_s1=rep_circulaire(mini_planete, 120, [120, 0, 10], 1/50)
Anneau_s2=rep_circulaire(mini_planete, 120, [120, 0, -10], 1/50)



#Planète colonisée
drapeau=fonction_drapeau(cylindre, triangle, 1)
drapeau_minion=minion_drapeau(drapeau, minion,20)
planete_col=planete_colonisee(drapeau_minion, planete)
#Satellite
satellite=fonction_satellite(cube, -300, 300,10) #Répétiton linéaire dans la fonction
satellite_rot=rotation(satellite,np.pi/4,[1,0,0])
#Galaxy 
galaxy=rep_perso(6, planete_col, planete, 2/3) #Répétition selon la suite de fibonacci

#fusion et export du fichier STL
scene=fusion((Anneau_p,Anneau_s1,Anneau_s2,galaxy,satellite_rot))
f,v,n=scene[0],scene[1],scene[2]
EcrireSTLASCII('scene_final_XX.stl', f, v, n)
