
start = [0,0]
end = [4,3]
 
m = 1 

def line_eq (x,m):
    return m*x

for x in range(start[0],end[0]+1):
    print(str(x)" and " + str(line_eq(x,1)))