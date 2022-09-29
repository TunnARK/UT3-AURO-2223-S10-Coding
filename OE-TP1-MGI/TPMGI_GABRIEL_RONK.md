OE TP1 Compte Rendu de D. GABRIEL-CALIXTE et A. RONK
---

Support de cours :
- [OE.TP1.Sujet.pdf](https://raw.githubusercontent.com/TunnARK/UT3-AURO-2223-S10-Dendron/main/vault/assets/OE.TP1.Sujet.pdf)
- [GitHub Repository](https://github.com/TunnARK/UT3-AURO-2223-S10-Coding/tree/master/OE-TP1-MGI)
- [Documentation scipy.optimize](https://docs.scipy.org/doc/scipy/tutorial/optimize.html)

---

# Calcul numérique du MGI d’un RRR par PNL

![](/assets/images/OE.TP1.Sujet-01.png)
![](/assets/images/OE.TP1.Sujet-02.png)
![](/assets/images/OE.TP1.Sujet-03.png)

## Notes Présentation TP

- incrémentation vs solution directe
- mgd nécessaire pour avoir $X_k$ 
    - idée est que X_k = X_0
    - mgd donné et jacobienne donnée
    - $\dot X = J \dot q$ donnée
- Newton
    - au lieu de calculer la hessiene
    - on va calculer le zero de $H(q)$

- Gradient
    - singularite = det de la jaco à 0
        - attention sbras tendu det de J est 0
    - afficher l'erreur et espére que l'erreur converge vers 0
    - attention gradient sensible p.r. au pas

- scipy
    - ici on cherche le modele alors qu'avant on nous l'a donné et il fallait faire l'algo (ici l'algo est donné)

## 1. Méthode de Newton

> ![](/assets/images/OE.TP1.Sujet-04.png)

La méthode de Newton est implémenté grâce au programme ci-dessous :

```python
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

# Evolution Erreur
print("Evolution de l erreur")
plt.plot(Hq)
plt.show()

# Visualisation de la position du bras
print("Position optimale trouvee avec Newton")
dessinRRR(np.radians(q))
```
On exécute une boucle `while()` dans laquelle le point suivant est calculé par :

$$
x_{k+1}=x_{k}+pas\times J^{-1}H
$$

On ajouter la norme de l'erreur d'optimisation au vecteur pour le tracé.

D'ailleurs, on obtient pour un pas de $0,1$ et une erreur $\epsilon=1\times 10^{-3}$, on obtient le profil de convergence :

![Newton_pas_01_100_it.png]



$$
\text{Figure : Evolution de l'erreur pour $\epsilon$=0,001 et pas=0,1}
$$

Notons que la boucle à été arrêté poar le critère $\epsilon$. En effet, on a réalisé que 82 itérations.

![shell_newton_pas01_100it.png]

$$
\text{Figure : Capture du shell après exécution de 82 itérations}
$$

Nous pouvons aussi noter l'nfluence des paramètres de la recherche du minimum.

* Tout d'abord, nous constatons que plus le pas est fin, plus la direction sera précise. Par conséquent, la trajectoire sera plus courte et donc le mouvement prendra moins de temps.


![Newton_pas_1_100_it.png]
$$
\text{Figure : Evolution de l'erreur pour un pas=0,1, $\epsilon$=0,001 et 100 itérations}
$$

*Ensuite, un nombre d'itérations trop faible ne garanti pas de trouver la solution. On peut consater que si nous imposons `NbIter=10` alors la solution n'a pas le temps de converger comme l'explicite la figure ci-dessous.

![Newton_pas01_10it.png]

$$
\text{Figure : Evolution de l'erreur pour un pas=0,1, $\epsilon$=0,001 et 10 itérations}
$$

* Enfin, le choix du point de départ influe sur la rapidité de la convergence. On prend par exemple, le bras tendu (configuration proche d'une singularité qui fait chuter le nombre de degrés de liberté local) comme point d'initialisation. Cela se traduit par la code suivant qui place tout les rotoïdes à un angle de 180° :

```python
# Intialisation
q=np.radians([+180,+180,+180])
```
Dans ce cas, on abouti à un erreur qui oscille autour du point minimisant $H(q)$.

![Newton_init_proche_singularite.png]
$$
\text{Figure : Evolution de l'erreur pour un initialisation bras tendu avec un pas=0,1, $\epsilon$=0,001 et 100 itérations}
$$

## 2. Méthode du Gradient

> ![](/assets/images/OE.TP1.Sujet-05.png)

L'implémentation de la métohde des gradient est réaslisée grâce au code suivant :

```python
## Methode des gradients

print("## METHODE DES GRADIENTS")

# Intialisation
q=qinit
print('La configuration de depart est',qinit)
i=0
j=0
Hq=[]
pas=0.5 # pas fixe
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
```
Dans ce code il est demande à l'utilisateur de saisir le nombre d'itérations souhaitées `NbIter`. Ensuite, on exécute une boucle `while()` dont la condition d'arrêt est atteinte si le on exède le nombre d'itérations maximal ou si l'écart entre le vecteur souhaité `Xbut` et le Modèle Géométrique Direct (MGD) au point courant `q` est en dessous d'un certain seuil `eps`. 

Au sein de cette boucle, on calcul la transposée de la matrice jacobienne pour la configuaration courante. La configuation suivante est mis à jour par `q=q+pas*np.dot(Jt,H)`. On rempli un vecteur contenant l'écart entre les MGD courants et le but. Enfin, on compte le nombre d'itérations `i` qui peut être plus faible que `NbIter`. 

Enfin, le module `numpy` permet de réaliser le tracé

Avec un `pas=0.5`, la solution ne converge pas et on obtient pour  `NbIter=100` l'évolution de l'erreur suivante.


![Gradient_pas_05_100_it.png]

$$

    \text{Figure : Evolution de l'erreur pour un pas de 0,5, 100 itérations $\epsilon$=0,001}

$$

Il nous faut alors choisir un pas plus fin afin d'obtenir une meilleur précision. On s'apperçoit q'un pas de `pas=0.2` permet une convergence de l'erreur mais cela créait toujours des oscillations.

C'est à partir de `pas=0.1` que l'on trouve une convergence de l'erreur en 100 itératons.

![Gradient_pas_01_100_it.png] 

$$

    \text{Figure : Evolution de l'erreur pour un pas de 0,1, 100 itérations $\epsilon$=0,001}

$$

Veuillez noter que l'algorithme réalise toujours 100 itérations.

## 3. Utilisation de scipy.optimize

> ![](/assets/images/OE.TP1.Sujet-06.png)
[Documentation scipy.optimize](https://docs.scipy.org/doc/scipy/tutorial/optimize.html)


L'implémentation avec `scipy.optimize` est rendu possible par le code source suivant :

```python
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
```
Cette méthode nécessite déclarer la fonction à optimiser. Ensuite, on passe en paramètre de `optimize.minimise()` la référence à cette fonction ainsi que le point d'initialisation.

La solution est récupérée par `X.x`. On peut aussi l'afficher à l'écran et la comparer à la valeur but.

On obtient la preuve après exécution :






## 4. Optimisation sous contrainte

### 4.1 Contrainte de butée

> ![](/assets/images/OE.TP1.Sujet-07.png)

..

### 4.2 Contrainte sur la solution

> ![](/assets/images/OE.TP1.Sujet-08.png)

..

## 5. Travail à rendre en fin de séance

> ![](/assets/images/OE.TP1.Sujet-09.png)

..

## Code Fourni

```python
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
```