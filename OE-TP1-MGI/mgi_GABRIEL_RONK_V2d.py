#################
# OE TP1 MGI
# Antoine RONK
# Damien GABRIEL CALIXTE
# Toutes les définitions de fonctions sont effectué en local
# Un seul fichier TPMGI_GABRIEL_RONK.py a executer sur Python 3.10.6
#################

## Libraries
import matplotlib.pyplot as plt
import numpy as np
import time
from scipy import optimize 

## MGD 
# Calcul du MGD du robot RRR
# INPUT:  q = vecteur de configuration (radian, radian, radian)
# OUTPUT: Xc = vecteur de situation = (x,y, theta)
#              x,y en mètre et theta en radian
def mgd(qrad):
    # Paramètres du robot
    l=[1,1,1]
    c1= np.cos(qrad[0])
    s1=np.sin(qrad[0])
    c12= np.cos(qrad[0]+qrad[1])
    s12=np.sin(qrad[0]+qrad[1])
    theta= qrad[0]+qrad[1]+qrad[2]
    c123=np.cos(theta)
    s123=np.sin(theta)
    x=l[0]*c1 + l[1]*c12 +l[2]*c123
    y=l[0]*s1 + l[1]*s12 +l[2]*s123
    Xd=[x,y,theta]
    return Xd
  
## Test de validation du MGD
# INPUT de q en degré
qdeg = [90, -90, 0]
qr = np.radians(qdeg)
Xd= mgd(qr)
print("X=", Xd[0], "Y = ", Xd[1], "Theta (deg)= ", np.degrees(Xd[2]))

## JACOBIENNE   
# Calcul de J(q) du robot RRR
# INPUT:  q = vecteur de configuration (radian, radian, radian)
# OUTPUT: jacobienne(q) analytique
def jacobienne(qrad):
    # Paramètres du robot
    l=[1,1,1]
    c1= np.cos(qrad[0])
    s1= np.sin(qrad[0])
    c12= np.cos(qrad[0]+qrad[1])
    s12=np.sin(qrad[0]+qrad[1])
    theta= qrad[0]+qrad[1]+qrad[2]
    theta= np.fmod(theta,2*np.pi)
    c123=np.cos(theta)
    s123=np.sin(theta) 
    # Remplissage Jacobienne
    Ja=np.array([[-(l[0]*s1 + l[1]*s12 +l[2]*s123), -(l[1]*s12 +l[2]*s123), -(l[2]*s123)], 
                [(l[0]*c1 + l[1]*c12 +l[2]*c123), (l[1]*c12 +l[2]*c123),  (l[2]*c123)], 
                 [1, 1, 1]])
    return Ja

###################################################################################
# Afin de donner une situation atteignable pour le robot,
# vous pouvez utiliser le mgd pour définir Xbut à partir d'une configuration en q
###################################################################################

## qbut est donné en degré
qbutdeg= np.asarray([-20, -90,  120])

## Calcul Xbut à partir de qbut
Xbut= np.asarray(mgd(np.radians(qbutdeg)))
# Print des resultats
print("Xbut=", Xbut[0], "Ybut = ", Xbut[1], "Theta but (deg)= ", np.degrees(Xbut[2]))

## Fct d'affichage 2D du robot dans le plan
def dessinRRR(q) :
    xA, yA = (0, 0)
    xB, yB = (np.cos(q[0]), np.sin(q[0]))
    xC, yC = (np.cos(q[0]) + np.cos(q[0]+q[1])), (np.sin(q[0]) + np.sin(q[0]+q[1]))
    xD, yD = (np.cos(q[0]) + np.cos(q[0]+q[1]) + np.cos(q[0]+q[1]+q[2])),(np.sin(q[0]) + np.sin(q[0]+q[1]) + np.sin(q[0]+q[1]+q[2]))
    X=[xA, xB,xC,xD]
    Y=[yA,yB,yC,yD]
    plt.plot([xA, xB], [yA, yB], color="orange", lw=10, alpha=0.5,
             marker="o", markersize=20, mfc="red")
    plt.plot([xB, xC], [yB, yC], color="orange", lw=10, alpha=0.5,
             marker="o", markersize=20, mfc="red")
    plt.plot(X,Y, color="orange", lw=10, alpha=0.5, marker="o", markersize=20, mfc="red")
    plt.axis('equal')
    plt.axis('off')
    plt.show()

## Affichage de qbutdeg
## Affichage de qbut
print("Affichage de la position du bras a la configuration but")
dessinRRR(np.radians(qbutdeg))



###################################################################################
# Boucle principale de calcul du MGI
###################################################################################



## Définition de Xbut à partir de qbutdeg en degré
qbutdeg= np.asarray([45.,45.,-60])
qbut = np.radians(qbutdeg)
Xbut= np.asarray(mgd(qbut))



## Définition de qinit 
qinitdeg=np.asarray([120., 25,  45.])
qinit= np.radians(qinitdeg)
Xinit=np.asarray(mgd(qinit))
print("Xinit = ",Xinit)
print("qinit = ",qinit)

## Methode de Newton

print("## METHODE DE NEWTON")

# Intialisation
q=qinit

print('La configuration de depart est',qinit)
i=0
j=0
Hq=[]
pas=0.1 # pas fixe
eps=0.001 # espilon
Hnorm = 1000

# Saisie NnIter
NbIter=int(input('Saisir le nombres d iterations : '))

# Boucle principale
while(Hnorm>eps and i<NbIter) :
    # Jacobienne
    J = jacobienne(q)
    # Calcul de l'inverse
    Jinv=np.linalg.inv(J)
    # Calcul de l ecart entre le Xbut et le Xc actuel
    H=(Xbut-mgd(q))
    # Iteration
    q=q+pas*np.dot(Jinv,H)
    # Calcul de l erreur avec norme euclidienne
    Hnorm=np.linalg.norm(H)
    # Remplissage du vecteur avec la norme
    Hq.append(Hnorm)
    i=i+1
    
# Info Boucle
print("La boucle a ete executee ",i,"fois")
# print("Xbut=", Xbut[0], "Ybut = ", Xbut[1], "Theta but (deg)= ", np.degrees(Xbut[2]))
# print("qbut=", qbut[0], "qbut = ", qbut[1], "Theta qbut (deg)= ", np.degrees(qbut[2]))

# Evolution Erreur
print("Evolution de l erreur")
plt.plot(Hq)
plt.show()

# Visualisation de la position du bras
print("Position optimale trouvee avec Newton")
dessinRRR(np.radians(q))

## Methode des gradients

print("## METHODE DES GRADIENTS")

# Intialisation
q=qinit
print('La configuration de depart est',qinit)
i=0
j=0
Hq=[]
pas=0.001 # pas fixe
eps=0.001 # espilon
Hnorm = 1000

# Saisie NnIter
NbIter=int(input('Saisir le nombres d iterations : '))

# Boucle principale
while(Hnorm>eps and i<NbIter) :
    # Jacobienne
    J = jacobienne(q)
    # Calcul de la transposee
    Jt=J.transpose()
    # Calcul de l ecart entre le Xbut et le Xc actuel
    H=(Xbut-mgd(q))
    # Iteration
    q=q+pas*np.dot(Jt,H)
    # Calcul de l erreur avec norme euclidienne
    Hnorm=np.linalg.norm(H)
    # Remplissage du vecteur avec la norme
    Hq.append(Hnorm)
    i=i+1
    
# Info Boucle
print("La boucle a ete executee ",i,"fois")
# print("Xbut=", Xbut[0], "Ybut = ", Xbut[1], "Theta but (deg)= ", np.degrees(Xbut[2]))
# print("qbut=", qbut[0], "qbut = ", qbut[1], "Theta qbut (deg)= ", np.degrees(qbut[2]))

# Evolution Erreur
print("Evolution de l erreur")
plt.plot(Hq)
plt.show()


# Visualisation de la position du bras
print("Position optimale trouvee avec la methode des gradients")
dessinRRR(np.radians(q))

## Methode des scipy.optimize

# Reinitialisation de la configuration initiale
q=qinit


# Definition de la fonction a minimiser (retourne une variable de dimension 1)

FuncH=lambda q: np.linalg.norm((Xbut-mgd(q)))

# Construction du point de depart de l algorithme

initial_guess=(Xbut-mgd(qinit))


#Optimisation avec minimize

X=optimize.minimize(FuncH, qinit)
print("L optimum donnee par scipy est :", mgd(X.x), "\n Le but etait de", Xbut)


# Visualisation de la position du bras
print("Position optimale trouvee avec scipy.minimize")
dessinRRR(np.radians(X.x))

## Methode optimale comprenant une approxiamtion de la jacobienne

# Initialisation de la configuration
q=qinit


# Definition de la fonction objectif (qui donne un resulat vectoriel a present)
FuncH=lambda q: (Xbut-mgd(q))

# Ititalisation et calcul du resulat de sortie : vecteur ligne de taille 3
Jh=optimize.approx_fprime(np.ones(3),FuncH)

print(Jh)

# Optimisation avec la jacobienne approximee
#X_japp=optimize.(FuncH,qinit,jac=Jh)

##
##nbpas= ???
##epsx= ???
## erx = valeur initale de du critère qu'on cherche à minimiser
##list_erreur = [erx] # 
##start_time = time.process_time()
##while (????):
##    direction = ???
##    pas= ???
##    ...
##    ...
##    list_erreur.append(erx) # Stocker la valeur du critère dans une liste
##    i=i+1
##print("--- %s seconds ---" % round(time.process_time() - start_time,6))
### Visualisation des résultats
##print("Valeur finale du critère =",??," après ",i," itérations")
##print("qinit en deg =", qinitdeg)
##print("qcfinal en deg", np.degrees(qc))
##X= mgd(qc)
##print("Xinit              =",Xinit, type(Xinit))
##print("Xfinal avec qfinal = ",X)
##print("Xbut à atteindre   =", Xbut, type(Xbut))
##Xer= Xbut -X
##erx=np.linalg.norm(Xer)
##print("Erreur finale=",erx)
##abs = np.linspace(0,len(list_erreur)-1,(len(list_erreur)))
##plt.plot(abs,list_erreur,'k')
##plt.show(block=True)
##
##dessinRRR(qc)
