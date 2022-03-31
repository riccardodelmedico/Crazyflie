# Crazyflie
Master Degree Project for Navigation and Guidance Systems class. 
Crazyflie repository made by Cioni, Del Medico, Gazzanelli

crazyfun__20220311_102238 - Kalman nostri, no correzioni
crazyfun__20220311_103259 - Kalman base

Costante di guida sempre pari a 5.

 Guida con Target Accelleratne (HE:=0)
APNG[Vd=0.75[m/s]; pos_in_t=[-1,1.5][m]; vel_in_t=[0,-0.55][m/s], acc_mod = 0.35 [m/s^2]]

    crazyfun__20220311_105351

Guida con target accellerante(HE:=0) PNG 
[Vd=0.55[m/s]; pos_in_t=[-1,1.5][m]; vel_in_t=[0,-0.5][m/s], acc_mod = 0.2 [m/s^2]]

    crazyfun__20220311_104854 

 Guida con target non accelerante (HE:= PI/4)  PNG
 [Vd=0.55[m/s]; pos_in_t=[-1,1.5][m]; vel_in_t=[0.3,-0.3][m/s]]
    
    crazyfun__20220311_104606

 Guida con target non accelerante (HE:=PI/2) APNG 
 [Vd=0.85[m/s]; pos_in_t=[-1,1.5][m]; vel_in_t=[0.5,0][m/s]]

    crazyfun__20220311_111140

 Guida con wand ferma (e R_interception = 1cm)
[Vd=0.65[m/s]]

    crazyfun__20220311_113929

 wand accelerante (HE:= 90) (e R_interception = 3cm)
[Vd=0.6[m/s]]

    crazyfun__20220311_140943

 wand accelerante (HE:= 90)  (e R_interception = 3cm)
[Vd=0.65[m/s]]

    crazyfun__20220311_150202

 wand accelerante (HE:= 90) (e R_interception = 3cm)
[Vd=0.65[m/s]]

    crazyfun__20220311_150250

wand accelerante (HE:= 0) (e R_interception = 5cm)
[Vd=0.65[m/s]]

    crazyfun__20220311_151252 
 wand accelerante (HE:= 90) (e R_interception = 5cm)
[Vd=0.8[m/s]]

    crazyfun__20220311_151434
wand: lineare, accelerante senza HE e con HE 
(ripetuta x2 la seconda batteria sono quelle di cui è stato fatto il video)[Vd=0.8[m/s]]

    crazyfun__20220311_160654
    crazyfun__20220311_160733
    crazyfun__20220311_160803

    crazyfun__20220311_161018
    crazyfun__20220311_161049
    crazyfun__20220311_161132

target virtuale accelerante:

HE 90 [Vd=0.8[m/s]; pos_in_t=[-1,0.5][m]; vel_in_t=[0.5,0][m/s], acc_mod = 0.15 [m/s^2]]:
    
    crazyfun__20220324_114627 APNG
    crazyfun__20220324_115108 PNG buona
    crazyfun__20220324_114931 PNG fallita
    crazyfun__20220324_115719 PNG fallita
    crazyfun__20220324_115942 PNG buona

HE 0 [Vd=0.8[m/s]; pos_in_t=[-1,2][m]; vel_in_t=[0,-0.5][m/s], acc_mod = 0.3 [m/s^2]]:

    crazyfun__20220324_101650 APNG buona
    crazyfun__20220324_101721 PNG buona
target virtuale non accelerante:
HE 90 [Vd=0.8[m/s]; pos_in_t=[-1,1.5][m]; vel_in_t=[0.5,0][m/s]]:

        crazyfun__20220324_120929 APNG buona
        crazyfun__20220324_121203 PNG buona
HE 45 [Vd=0.8[m/s]; pos_in_t=[-1,2][m]; vel_in_t=[0.33,-0.33][m/s]]:

    crazyfun__20220324_122048 PNG buona Chase_vel=0.6[m/s]

HE -45 [Vd=0.45[m/s]; pos_in_t=[1,2][m]; vel_in_t=[-0.33,-0.33][m/s]] :

    crazyfun__20220324_122627 PNG buona Chase_vel=0.6[m/s]
        
