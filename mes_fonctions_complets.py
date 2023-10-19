# -*- coding: utf-8 -*-
"""
Created on Thu Oct  5 20:32:29 2023

@author: flore
"""

# NOTE: Pour toutes les fonctions, on prend en entrée des objets sous forme de listes [F, V, N] et on retourne les objets modifiés sous la même forme.

import numpy as np
from MEC1315_STL import *

# fonction pour homothétie, grandissement doit être float
def homothetie(objet, grandissement):
    objet_mod = objet.copy()
    objet_mod[1] = grandissement * objet_mod[1]
    return objet_mod

# fonction pour translation, déplacement doit être array de format [1,3]
def translation(objet, deplacement):
    objet_mod = objet.copy()
    objet_mod[1] = objet_mod[1] + deplacement
    return objet_mod
    
# fonction pour centrer un objet en x et en y et en z=0
def centrer(objet):
    F, V, N = objet[0].copy(), objet[1].copy(), objet[2].copy()
    centre_x = (min(V[:,0]) + max(V[:,0]))/2
    centre_y = (min(V[:,1]) + max(V[:,1]))/2
    min_z = min(V[:,2])
    objet_centre = translation(objet, np.array([-centre_x,-centre_y,-min_z]))
    return objet_centre

# fonction de rotation
def rotation(objet, angle_rotation, axe_rotation): # axe de rotation de la forme [1, 0, 0] et angle de rotation en radians
    
    if axe_rotation==[1, 0, 0]:
        R=Rx(angle_rotation)
    elif axe_rotation==[0, 1, 0]:
        R=Ry(angle_rotation)
    elif axe_rotation==[0, 0, 1]:
        R=Rz(angle_rotation)
        
    objet_rotation = [objet[0], objet[1].dot(R), objet[2].dot(R)]
    
    return objet_rotation

# change dimension d'un objet en appliquant les coefficients a, b et c sur les composantes x, y et z
def affinite_vectorielle(objet, a, b, c):
    F, V, N = objet[0].copy(), objet[1].copy(), objet[2].copy()
    V[ :,0] = V[ :,0]*a
    V[ :,1] = V[ :,1]*b
    V[ :,2] = V[ :,2]*c
    N = CalculNormal( F, V )
    objet_final = [F, V, N]
    return objet_final

# fonction pour fusionner, objets contient tous les objets individuels à fusionner
def fusion(objets): 
    objets_fusionnes = [np.empty([0,3]),np.empty([0,3]),np.empty([0,3])] # création des arrays F, V et N finaux
    i = 0 # initialisation du compteur
    for objet_individuel in objets: # on prend F, V et N de chaque objet à fusionner
        if i == 0:
            objets_fusionnes[0] = np.vstack((objets_fusionnes[0], objet_individuel[0])) # ajout du premier objet à la matrice vide objets_fusionnes[0]
        else:
            nb_vertex = len(objets_fusionnes[1]) # on détermine le nombre de vertex total des objets déjà ''fusionnés''
            objets_fusionnes[0] = np.vstack((objets_fusionnes[0], objet_individuel[0]+nb_vertex)) # concaténation et ajout de nb_vertex sur objets_fusionnes[0]
            
        objets_fusionnes[1] = np.vstack((objets_fusionnes[1], objet_individuel[1])) # concaténation
        objets_fusionnes[2] = np.vstack((objets_fusionnes[2], objet_individuel[2])) # concaténation
        i += 1

    return objets_fusionnes

# fonction pour répétition circulaire, nb_rep correspond au nombre de répétitions souhaité (doit être int)
def rep_circulaire(objet, nb_rep, deplacement, grandissement):
    objet_final = [np.empty([0,3]), np.empty([0,3]), np.empty([0,3])] # création des arrays F, V et N finaux
    nb_vertex = len(objet[1]) # on détermine le nombre de vertex de l'objet original
    objet = homothetie(objet, grandissement)
    objet = translation(objet, deplacement)
    for i in range(nb_rep):
        theta = 2*np.pi/nb_rep*i # on trouve l'angle pour la répétition i
        R = Rz(theta) # matrice de rotation par rapport à l'axe des z
        
        objet_i = objet # créations de arrays F, V et N pour l'itération i
        
        objet_final[0] = np.vstack((objet_final[0], objet_i[0] + nb_vertex*i)) # concaténation et ajout de nb_vertex*i sur objet_i[0]
        objet_final[1] = np.vstack((objet_final[1], objet_i[1].dot(R))) # Rotation et concaténation
        objet_final[2] = np.vstack((objet_final[2], objet_i[2].dot(R))) # Rotation concaténation
        
    return objet_final
    
# fonction pour répétition rectiligne
def repetition_rectiligne(objet, nb_rep, espacement):
    objet_final = [np.empty([0,3]), np.empty([0,3]), np.empty([0,3])] # création des arrays F, V et N finaux
    nb_vertex = len(objet[1]) # on détermine le nombre de vertex de l'objet original
    
    for i  in range (nb_rep):
        objet[1][ :,0] = objet[1][ :,0] + espacement
        
        objet_final[0] = np.vstack((objet_final[0], objet[0] + nb_vertex*i)) # concaténation et ajout de nb_vertex*i sur F_i
        objet_final[1] = np.vstack((objet_final[1], objet[1])) # concaténation
        objet_final[2] = np.vstack((objet_final[2], objet[2])) # concaténation
        
    return objet_final

# fonction pour notre répétition personnalisée de la suite de Fibonacci
def rep_perso(nb_rep, autre_objet, objet_central, grandissement_central): 
    objets = [homothetie(objet_central, grandissement_central)]
 
    #suite de fibonacci dans les 4 quadrants
    i = 0
    j = 1
    a = 0
    res = np.array([0])
 
    ppmm = np.array([1,1,-1,-1])
    ppmmtot = np.array([1,1,-1,-1])
    pmmp = np.array([1,-1,-1,1])
    pmmptot = np.array([1,-1,-1,1])
 
     # matrice pour créer la rotation dans les 4 quadrants
    while len(pmmptot) < nb_rep: #Pour créer la suite +--+ pour x
         pmmptot = np.hstack([pmmptot,pmmp])
    while len(ppmmtot) < nb_rep: #Pour créer la suite ++-- pour y
         ppmmtot = np.hstack([ppmmtot,ppmm])
     
    # suite de fibonnacci     
    while a < nb_rep: 
         k = np.array([i+j])
         res = np.hstack([res,k])
         j = i
         i = res[-1]
         a += 1
     
    x,y,z=0,0,0

    for o in range(1, nb_rep):
        xi = res[o+1]*pmmptot[o]
        x=np.hstack((x,xi))
        yi = res[o+1]*ppmmtot[o]
        y=np.hstack((y,yi))
        zi=0
        z=np.hstack((z,zi))
    position=np.vstack((x,y,z)).T

     # placement des planètes sur la suite de fibonacci
    objet_final = [np.empty([0,3]), np.empty([0,3]), np.empty([0,3])]
    nb_vertex = len(autre_objet[1])
    m=nb_rep+1
    for i in range(nb_rep):
        objet=autre_objet
    
    
     
        objet_final[0] = np.vstack((objet_final[0], objet[0]+ nb_vertex*i)) # concaténation et ajout de nb_vertex*i sur objet_i[0]
        objet_final[1] = np.vstack((objet_final[1], objet[1]*i/m + 110*position[i])) 
        objet_final[2] = np.vstack((objet_final[2], objet[2])) 
    return objet_final


# fonction de répétition personnalisée en spirale
def spirale_perso(mini_planete,nbre_planete,nbre_planete_secondaire):
    nb_rep=nbre_planete
    nb_seconde_planete=nbre_planete_secondaire
    mini=mini_planete.copy()
    nb_passage=nb_rep-1
    thmax=90*nb_passage
    nb_passage_planete=int(nb_seconde_planete/nb_passage)
    int_th=thmax/nb_seconde_planete

    i = 0
    j = 1
    a = 0
    res = np.array([0])

    ppmm = np.array([1,1,-1,-1])
    ppmmtot = np.array([1,1,-1,-1])
    pmmp = np.array([1,-1,-1,1])
    pmmptot = np.array([1,-1,-1,1])

     # matrice pour créer la rotation dans les 4 quadrants
    while len(pmmptot) < nb_rep: #Pour créer la suite +--+ pour x
         pmmptot = np.hstack([pmmptot,pmmp])
    while len(ppmmtot) < nb_rep: #Pour créer la suite ++-- pour y
         ppmmtot = np.hstack([ppmmtot,ppmm])

    while a < nb_rep: 
         k = np.array([i+j])
         res = np.hstack([res,k])
         j = i
         i = res[-1]
         a += 1


    allo=res[1:]
    racine=[]
    x=[]
    y=[]
    px=[]
    py=[]
    for i in range(len(allo)):
        a=np.sqrt(2*allo[i]**2)
        racine.append(a)
    for o in range(len(allo)):
        xi = racine[o]*pmmptot[o]
        x.append(xi)
        yi = racine[o]*ppmmtot[o]
        y.append(yi)
    for i in range(len(allo)):
        if i % 2==0:
            px.append(x[i])
            py.append(0)
        else:
            py.append(y[i])
            px.append(0)
    

    nb_vertexmini=len(mini[1])

    mini_final=[np.empty([0,3]), np.empty([0,3]), np.empty([0,3])]
    i=0
    j=0
    ptot=[]
    atot=[]
    btot=[]
    while i<(len(racine)-1):
        
        a=abs(px[i+1]-px[i])
        b=abs(py[i+1]-py[i])
        at=a.copy()
        bt=b.copy()
        for j in range(nb_passage_planete):
            atot.append(at)
            btot.append(bt)
        i=i+1

    xp=[]
    yp=[]
    zp=[]
    mini=homothetie(mini, 1/10)
    
    for i in range(nb_seconde_planete):
        th=int_th*i*np.pi/180
        xp.append(atot[i]*np.cos(th)) 
        yp.append(btot[i]*np.sin(th)) 
        zp.append(0)
    po=np.vstack([xp,yp,zp]).T
    nb_vertexmini=len(mini[1])

    mini_final=[np.empty([0,3]), np.empty([0,3]), np.empty([0,3])]
    for i in range(nb_seconde_planete):
        objet=mini
        
        mini_final[0] = np.vstack((mini_final[0], objet[0]+ nb_vertexmini*i)) # concaténation et ajout de nb_vertex*i sur objet_i[0]
        mini_final[1] = np.vstack((mini_final[1], objet[1] + 110*po[i])) 
        mini_final[2] = np.vstack((mini_final[2], objet[2])) 
    mini_final=rotation(mini_final, 1/4*np.pi, [0,0,1])
    return mini_final


def étoiles_partout(objet):
    
    x = 5*np.array([200, 100, 300, 450, 350, 50, 900, 550, 250, 600])
    y = 5*np.array([100, 500, 200, 650, 480, 654, 345, 678, 543, 900])
    z = 5*np.array([600, 200, 340, 657, 789, 543, 567, 977, 123, 456])

    coordonnees = [[1, 1, 1],
                   [-1, 1, 1],
                   [-1, -1, 1],
                   [-1, -1, -1],
                   [1, -1, -1],
                   [1, 1 , -1],
                   [1, -1, 1],
                   [-1, 1, -1]]



    étoiles = [np.empty([0,3]), np.empty([0,3]), np.empty([0,3])]
    for cadran in coordonnees:
        for n in range(0, len(x)):
            étoiles_n = translation(objet, [cadran[0]*x[n], cadran[1]*y[n], cadran[2]*z[n]])
            étoiles = fusion([étoiles, étoiles_n])
    return étoiles
