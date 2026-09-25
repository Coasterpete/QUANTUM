f = open('tests/CMakeLists.txt', 'r')
content = f.read()
f.close()
idx = content.find('QuantumCoreSupportSerializationTests')
while idx >= 0:
    print('Found at:', idx)
    idx = content.find('QuantumCoreSupportSerializationTests', idx+1)