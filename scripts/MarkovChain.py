
import numpy as np

def generateTable(data,k=3):
    
    T = {}
    for i in range(len(data)-k):
        X = data[i:i+k]
        X_hash = str(X).strip('[]')
        Y = data[i+k]
        print("|")
        print(Y)
        #print("X  %s and Y %s  "%(X,Y))
        
        if T.get(X_hash) is None:
            T[X_hash] = {}
            T[X_hash][Y] = 1
        else:
            if T[X_hash].get(Y) is None:
                T[X_hash][Y] = 1
            else:
                T[X_hash][Y] += 1
    
    return T

def convertFreqIntoProb(T):     
    for kx in T.keys():
        s = float(sum(T[kx].values()))
        for k in T[kx].keys():
            T[kx][k] = T[kx][k]/s
                
    return T

def sample_next(ctx,model,k):
     
    ctx = ctx[-k:]
    ctx = str(ctx).strip('[]')
    if model.get(ctx) is None:
        return " "
    possible_Chars = list(model[ctx].keys())
    possible_values = list(model[ctx].values())
    possible_chars_idx = []
    array_possible_poses = np.array(possible_Chars)
    for i in range (len(array_possible_poses)):
        possible_chars_idx.append(i)
    array_possible_values = np.array(possible_values)
    print(array_possible_poses)
    print(possible_values)
    idx = np.random.choice(possible_chars_idx,p=array_possible_values)
    return possible_Chars[idx]

listOfPoses  = [(1,0,0), (2,0,0), (3,0,0), (4,0,0), (5,0,0), (6,0,0), (7,0,0)]
sequence = ""
TestData = [(1,0,0),(2,0,0),(4,0,0)]



T = generateTable(listOfPoses)
T = convertFreqIntoProb(T)
print(T)

print(sample_next(TestData, T, 3))
