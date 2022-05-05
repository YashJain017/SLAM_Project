import numpy as np
import pandas as pd
import json
import pickle

def generateTable(data,k=3):
    
    T = {}
    for i in range(len(data)-k):
        X = data[i:i+k]
        str1=""
        for j in range(k):
            str1=str1+"("
            for m in range(3):
                str1=str1+"{:.1f}".format(X[j][m])+", "
            str1=str1.strip(", ")+"), "
        X_hash = str1.strip(", ")
        
        Y = tuple(np.around(data[i+k],decimals=1))
        
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
    idx = np.random.choice(possible_chars_idx,p=array_possible_values)
    return possible_Chars[idx]

listOfPoses  = [(1,0,0), (2,0,0), (3,0,0), (4,0,0), (5,0,0), (6,0,0), (7,0,0)]
sequence = ""
TestData = [(1,0,0),(2,0,0),(4,0,0)]

df=pd.read_csv(r"path3_15combined.csv") #FILEPATH
df=df.round(decimals=1)

X=np.array(df)
X=np.around(X,decimals=1)

T = generateTable(X)
T = convertFreqIntoProb(T)

with open('model_path3_15_combined.pkl', 'wb') as f:
    pickle.dump(T, f)
print("Finished Saving Model")

print("Loading Model")
with open('model_path3_15_combined.pkl', 'rb') as f:
    data = pickle.load(f)
print(data)