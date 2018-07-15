import sys
def calcDist(posA, posB):
    return abs(posA[0] - posB[0]) + abs(posA[1] - posB[1])

def calcSteps(name):
    """
    @!: ABC DEF
    GHI JKL MNO
    PQRS TUV WXYZ
    :param name:
    :return:
    """
    posDict = {
        'J': (1, 1),
        'K': (1, 1),
        'L': (1, 1),
        'A': (0, 1),
        'B': (0, 1),
        'C': (0, 1),
        'P': (2, 0),
        'Q': (2, 0),
        'R': (2, 0),
        'S': (2, 0),
        'T': (2, 1),
        'U': (2, 1),
        'V': (2, 1),
        'M': (1, 2),
        'N': (1, 2),
        'O': (1, 2),
        'W': (2, 2),
        'X': (2, 2),
        'Y': (2, 2),
        'Z': (2, 2),
        '@': (0, 0),
        '!': (0, 0),
        ':': (0, 0),
        'G': (1, 0),
        'H': (1, 0),
        'I': (1, 0),
        'D': (0, 2),
        'E': (0, 2),
        'F': (0, 2),
    }
    lastC = "*"
    dist = 0
    for c in name:
        if lastC != "*":
            dist += calcDist(posDict[lastC], posDict[c])
        else:
            dist += calcDist(posDict["@"], posDict[c])
        lastC = c
    return dist

if __name__ == "__main__":
    sys.stdin = open("input.txt", "r")
    T = int(sys.stdin.readline().strip())
    for i in range(T):
        name = sys.stdin.readline().strip()
        print(calcSteps(name))
    #dx = {"@!:":(0, 0), "ABC":(0, 1),  "DEF":(0, 2),
    #"GHI":(1, 0), "JKL":(1, 1), "MNO":(1, 2),
    #"PQRS":(2, 0), "TUV":(2, 1), "WXYZ":(2, 2)}
    #for k in dx:
    #    for c in k:
    #        print("'{0}':{1},".format(c, dx[k]))

