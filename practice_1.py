# try:
#     4 / 0
# except ZeroDivisionError as e:
#     print(e)

# try:
#     4 / 0
# except:
#     print('dd')

# a = [1,2,3,4]
# while a:
#     a.pop()
#     print(a)

a = [1,2,3]
b = a[:]

a[1] = 4
print(a, b)