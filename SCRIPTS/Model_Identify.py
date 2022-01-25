import numpy as np
#quando si definisce il modello ci si deve ricordare di utilizzare già il modello invertito in forma causale se necessario aggiungendo un polo in alta frequenza
class Model_Compensation:
    def __init__(self,den=np.array([1.0]),num= np.array([1.0])):
        # [p1, p2, p3,...] === p1 + p2*z^(-1) + p3* z^(-2)+...
        self.num = num
        self.den = den
        self.old_input = np.zeros(len(self.num)-1)
        self.old_output = np.zeros(len(self.den)-1)
    def compensation(self,omega_des):
        output_omega = (self.num.dot(np.append(omega_des,self.old_input)) - self.den[1:len(self.num)].dot(self.old_output))/self.den[0]
        for i in np.arange(len(self.num)-2,-1,-1):
            if i == 0:
                self.old_input[0] = omega_des
                self.old_output[0] = output_omega
            else:
                self.old_input[i] = self.old_input[i-1]
                self.old_output[i] = self.old_output[i - 1]
        return output_omega