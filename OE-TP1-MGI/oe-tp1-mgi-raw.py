#############
# Code à utiliser pour débuter votre TP
#################
import matplotlib.pyplot as plt
import numpy as np
import time

#################################################
# Calcul du MGD du robot RRR
# INPUT: q = vecteur de configuration (radian, radian, radian)
# OUTPUT: Xc = vecteur de situation = (x,y, theta)
# x,y en mètre et theta en radian
def mgd(qrad):
#### Paramètres du robot
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
#################################################
# test de validation du MGD
##### INPUT de q en degré ###
qdeg = [90, -90, 0]
qr = np.radians(qdeg)
Xd= mgd(qr)
print("X=", Xd[0], "Y = ", Xd[1], "Theta (deg)= ", np.degrees(Xd[2]))

#################################################
# Calcul de J(q) du robot RRR
# INPUT: q = vecteur de configuration (radian, radian, radian)
# OUTPUT: jacobienne(q) analytique
def jacobienne(qrad):
#### Paramètres du robot
    l=[1,1,1]
    c1= np.cos(qrad[0])
    s1= np.sin(qrad[0])
    c12= np.cos(qrad[0]+qrad[1])
    s12=np.sin(qrad[0]+qrad[1])
    theta= qrad[0]+qrad[1]+qrad[2]
    theta= np.fmod(theta,2*np.pi)
    c123=np.cos(theta)
    s123=np.sin(theta)
    Ja=np.array([[-(l[0]*s1 + l[1]*s12 +l[2]*s123), -(l[1]*s12 +l[2]*s123), -(l[2]*s123)], [(l[0]*c1 + l[1]*c12 +l[2]*c123), (l[1]*c12 +l[2]*c123), (l[2]*c123)], [1, 1, 1]])
    return Ja

###################################################################################
# Afin de donner une situation atteignable pour le robot,
# vous pouvez utiliser le mgd pour définir Xbut à partir d’une configuration en q
###################################################################################
## qbut est donné en degré
qbutdeg= np.asarray([45, 45, -60.])
## Calcul Xbut à partir de qbut
Xbut= np.asarray(mgd(np.radians(qbutdeg)))
print("Xbut=", Xbut[0], "Ybut = ", Xbut[1], "Theta but (deg)= ", np.degrees(Xbut[2]))

############################################
## Fct d’affichage 2D du robot dans le plan
def dessinRRR(q) :
xA, yA = (0, 0)
xB, yB = (np.cos(q[0]), np.sin(q[0]))
xC, yC = (np.cos(q[0]) + np.cos(q[0]+q[1])), (np.sin(q[0]) + np.sin(q[0]+q[1]))
xD, yD = (np.cos(q[0]) + np.cos(q[0]+q[1]) + np.cos(q[0]+q[1]+q[2])),(np.sin(q[0]) + np
X=[xA, xB,xC,xD]
Y=[yA,yB,yC,yD]
plt.plot(X,Y, color="orange", lw=10, alpha=0.5, marker="o", markersize=20, mfc="red")
plt.axis(’equal’)
plt.axis(’off’)
plt.show()
################################################
############ Exemple d’affichage
dessinRRR(np.radians(qbutdeg))

########################################
######
###### Boucle principale de calcul du MGI
######################################
####
#### Définition de Xbut à partir de qbutdeg en degré
qbutdeg= np.asarray([45.,45.,-60])
qbut = np.radians(qbutdeg)
Xbut= np.asarray(mgd(qbut))
dessinRRR(qbut)
#### Définition de qinit
qinitdeg=np.asarray([120., 25, 45.])
qinit= np.radians(qinitdeg)
Xinit=np.asarray(mgd(qinit))
print("Xinit = ",Xinit)

##### A CODER
##q= qinit
##i=0
##
##
##nbpas= ???
##epsx= ???
## erx = valeur initale de du critère qu’on cherche à minimiser
##list_erreur = [erx] #
##start_time = time.process_time()
##while (????):
## direction = ???
## pas= ???
## ...
## ...
## list_erreur.append(erx) # Stocker la valeur du critère dans une liste
## i=i+1
##print("--- %s seconds ---" % round(time.process_time() - start_time,6))
### Visualisation des résultats
##print("Valeur finale du critère =",??," après ",i," itérations")
##print("qinit en deg =", qinitdeg)
##print("qcfinal en deg", np.degrees(qc))
##X= mgd(qc)
##print("Xinit =",Xinit, type(Xinit))
##print("Xfinal avec qfinal = ",X)
##print("Xbut à atteindre =", Xbut, type(Xbut))
##Xer= Xbut -X
##erx=np.linalg.norm(Xer)
##print("Erreur finale=",erx)
##abs = np.linspace(0,len(list_erreur)-1,(len(list_erreur)))
##plt.plot(abs,list_erreur,’k’)
##plt.show(block=True)
##
##dessinRRR(qc)