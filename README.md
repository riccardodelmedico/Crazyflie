# Crazyflie
Master Degree Project for Navigation and Guidance Systems class. 
Crazyflie repository made by Cioni, Del Medico, Gazzanelli

### Estimation Test
1) Default parameters with angles correction:
     
   
    crazyfun__20220315_110052
2) Modified parameters with angles correction:


    crazyfun__20220315_110420

3) Modified parameters without angles correction:
 

    crazyfun__20220315_110817
4) Yaw-test with angles correction:
  

    crazyfun__20220317_101010
5) Yaw-test without angles correction:
    

    crazyfun__20220317_100512

### Guidance Test (N = 5)
- **_TARGET VIRTUALE_**
1)  Guida APNG Target Accelerante (HE = 0) \
    (Vd=0.75[m/s]; pos_in_t=[-1,1.5][m]; vel_in_t=[0,-0.55][m/s], acc_mod = 0.35 [m/s^2])


    crazyfun__20220311_105351

2) Guida PNG Target Accelerante (HE = 0) \
   (Vd=0.55[m/s]; pos_in_t=[-1,1.5][m]; vel_in_t=[0,-0.5][m/s], acc_mod = 0.2 [m/s^2])


    crazyfun__20220311_104854 

3)  Guida PNG Target Non Accelerante (HE = PI/4) \ 
    (Vd=0.55[m/s]; pos_in_t=[-1,1.5][m]; vel_in_t=[0.3,-0.3][m/s])
   
 
    crazyfun__20220311_104606

4) Guida APNG Target Non Accelerante (HE = PI/2)\
 (Vd=0.85 [m/s]; pos_in_t=[-1,1.5][m]; vel_in_t=[0.5,0][m/s])


    crazyfun__20220311_111140

5) Altri test Target Accelerante

   (HE=90) \
   [Vd=0.8[m/s]; pos_in_t=[-1,0.5][m]; vel_in_t=[0.5,0][m/s], acc_mod = 0.15 [m/s^2]]:
   
 
    crazyfun__20220324_114627 APNG
    crazyfun__20220324_115108 PNG buona
    crazyfun__20220324_114931 PNG fallita
    crazyfun__20220324_115719 PNG fallita
    crazyfun__20220324_115942 PNG buona
    

   (HE=0) \
   [Vd=0.8[m/s]; pos_in_t=[-1,2][m]; vel_in_t=[0,-0.5][m/s], acc_mod = 0.3 [m/s^2]]:

    crazyfun__20220324_101650 APNG buona
    crazyfun__20220324_101721 PNG buona

6) Altri test Target Non Accelerante 

   (HE=90) \
   [Vd=0.8[m/s]; pos_in_t=[-1,1.5][m]; vel_in_t=[0.5,0][m/s]]:

        crazyfun__20220324_120929 APNG buona
        crazyfun__20220324_121203 PNG buona


  (HE=45) \
  [Vd=0.8[m/s]; pos_in_t=[-1,2][m]; vel_in_t=[0.33,-0.33][m/s]]:

    crazyfun__20220324_122048 PNG buona Chase_vel=0.6[m/s]

 (HE=-45) \
 [Vd=0.45[m/s]; pos_in_t=[1,2][m]; vel_in_t=[-0.33,-0.33][m/s]] :

    crazyfun__20220324_122627 PNG buona Chase_vel=0.6[m/s]
- **_TARGET REALE_**

Wand Ferma (e R_interception = 1cm)
[Vd=0.65[m/s]]

    crazyfun__20220311_113929

 Wand Accelerante (HE = 90) (e R_interception = 3cm)
[Vd=0.6[m/s]]

    crazyfun__20220311_140943

Wand Accelerante (HE = 90)  (e R_interception = 3cm)
[Vd=0.65[m/s]]

    crazyfun__20220311_150202

 Wand Accelerante (HE = 90) (e R_interception = 3cm)
[Vd=0.65[m/s]]

    crazyfun__20220311_150250

Wand Accelerante (HE = 0) (e R_interception = 5cm)
[Vd=0.65[m/s]]

    crazyfun__20220311_151252 
Wand Accelerante (HE = 90) (e R_interception = 5cm)
[Vd=0.8[m/s]]

    crazyfun__20220311_151434

Wand: lineare, accelerante senza HE e con HE 
(ripetuta x2 la seconda batteria sono quelle di cui è stato fatto il video)[Vd=0.8[m/s]]

    crazyfun__20220311_160654
    crazyfun__20220311_160733
    crazyfun__20220311_160803

    crazyfun__20220311_161018
    crazyfun__20220311_161049
    crazyfun__20220311_161132


        
