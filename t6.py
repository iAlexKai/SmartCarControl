import sys

def hdiv(dividend, divisor, precision=0):
    """

    @author: cidplp

    @param dividend:
    @type dividend:int
    @param divisor:
    @type divisor:int
    @param precision:
    @type precision:int
    @return:
    @rtype:str
    """

    if isinstance(precision, int) == False or precision < 0:
        print('xx')
        return

    a = dividend
    b = divisor

    # 1
    if abs(a + b) == abs(a) + abs(b):
        flag = 1
    else:
        flag = -1

    # 2
    a = abs(a)
    b = abs(b)

    quotient = a // b
    remainder = a % b

    if remainder == 0:
        return quotient

    ans = str(quotient) + '.'

    i = 0
    while i < precision:
        a = remainder * 10
        quotient = a // b
        remainder = a % b
        ans += str(quotient)
        if remainder == 0:
            break
        i += 1

    if precision == 0:
        ans = ans.replace('.', '')

    if flag == -1:
        ans = '-' + ans

    return ans

def getRes(a, b, s):
    strRes = hdiv(a, b, 1000005)
    strRes = strRes.split(".")[1] if "." in strRes else "0"
    if len(strRes) < 1000000:
        strRes = strRes + ('0' * (1000003 - len(strRes)))
    findIdx = strRes.find(s)
    return -1 if findIdx == -1 else findIdx + 1
if __name__ == "__main__":
    sys.stdin = open("input.txt", "r")
    a, b = sys.stdin.readline().strip().split()
    a = int(a)
    b = int(b)
    s = sys.stdin.readline().strip()
    print(getRes(a, b, s))
