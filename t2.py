import sys

def getNextNzero(arr, curIdx):
    N = len(arr)
    i = curIdx
    while i < N and arr[i] == 0:
        i += 1
    return i

def calcDist(src, dst):
    sumSteps = 0
    N = len(src)

    # set start position
    #print(dst)
    #print(src)
    dstLeftIdx = getNextNzero(dst, 0)
    srcLeftIdx = getNextNzero(src, dstLeftIdx)

    #print(dstLeftIdx, srcLeftIdx)

    while dstLeftIdx < N and srcLeftIdx < N and dstLeftIdx <= srcLeftIdx:
        if dst[dstLeftIdx] > src[srcLeftIdx]:
            sumSteps += (srcLeftIdx - dstLeftIdx) * src[srcLeftIdx]
            dst[dstLeftIdx] -= src[srcLeftIdx]
            src[srcLeftIdx] = 0
            # update srcLeftIdx
            srcLeftIdx = getNextNzero(src, srcLeftIdx)
        elif dst[dstLeftIdx] < src[srcLeftIdx]:
            sumSteps += (srcLeftIdx - dstLeftIdx) * src[srcLeftIdx]
            src[srcLeftIdx] -= dst[dstLeftIdx]
            dst[dstLeftIdx] = 0
            dstLeftIdx = getNextNzero(dst, dstLeftIdx)
            srcLeftIdx = getNextNzero(src, dstLeftIdx)
        else:
            sumSteps += (srcLeftIdx - dstLeftIdx) * src[srcLeftIdx]
            dst[dstLeftIdx] = 0
            src[srcLeftIdx] = 0
            dstLeftIdx = getNextNzero(dst, dstLeftIdx)
            srcLeftIdx = getNextNzero(src, dstLeftIdx)
    if sum(src) == 0:
        return sumSteps

    leftNum = sum(src)

    srcLeftMostIdx = getNextNzero(src, 0)
    dstLeftMostIdx = getNextNzero(dst, 0)
    middlePartSum = leftNum * (srcLeftMostIdx + dstLeftMostIdx)
    leftDelta = sum([(i-srcLeftMostIdx) * src[i] for i in range(srcLeftMostIdx, N)])
    rightDelta = sum([(i-dstLeftMostIdx) * dst[i] for i in range(dstLeftMostIdx, N)])
    sumSteps += (middlePartSum + leftDelta + rightDelta)
    return sumSteps

if __name__ == "__main__":
    sys.stdin = open("input.txt", "r")
    N = int(sys.stdin.readline().strip())
    arr = []
    for i in range(2):
        line = sys.stdin.readline()
        arr.append([int(i) for i in line.strip().split()])
    print(calcDist(arr[0], arr[1]))
