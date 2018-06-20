import numpy as np

NUMBER_OF_THRUSTERS = 8
E = []

#We calculate this dynamically so we can use different COGs for submarine changes
def calculate_E_inverse():
    #Origin to Thruster vector
    Origin_to_Thruster= [
        [-3.606, -3.606, -30.394, -30.394, 3.063, -37.063, -15.391, -15.391],
        [-2.913, 19.413, -2.913, 19.413, 8.25, 8.25, -2.913, 19.413],
        [-0.3, -0.3, -0.3, -0.3, -1.0, -1.0, 3.7, 3.7]
    ]

    #Origin to Center of Gravity
    Origin_to_COG = [-18.305, 8.217, 1.796]

    #Thruster to Center of Gravity
    Thruster_to_COG = []

   
    #D is for each dimension
    for D in range(3):
        temp = []
        #T is for each thruster
        for T in range(NUMBER_OF_THRUSTERS):
            temp.append(Origin_to_Thruster[D][T] - Origin_to_COG[D])
        Thruster_to_COG.append(temp)

    Thruster_Direction_Vectors = [
        [1,1,1,1,0,0,0,0],
        [0,0,0,0,1,1,0,0],
        [0,0,0,0,0,0,1,1]   
    ]

    

    E = Thruster_Direction_Vectors
    E.append([])
    E.append([])
    E.append([])
    #T is for each thruster
    for T in range(NUMBER_OF_THRUSTERS):
        direction_vector = [[Thruster_Direction_Vectors[0][T]], [Thruster_Direction_Vectors[1][T]], [Thruster_Direction_Vectors[2][T]]] 
        cog_vector = [[Thruster_to_COG[0][T]], [Thruster_to_COG[1][T]], [Thruster_to_COG[2][T]]] 
         
        cross = np.cross(cog_vector, direction_vector, axis=0)
        for D in range(3):
            E[D+3].append(float(cross[D]))

    inverse_E = np.linalg.pinv(E)
    print np.shape(inverse_E)
    return np.matmul(inverse_E, E)

print calculate_E_inverse()           
