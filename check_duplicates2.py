f = open('tests/CMakeLists.txt', 'r')
content = f.read()
f.close()
idx = content.find('QuantumCoreSupportSerializationTests')
while idx >= 0:
    print('Found at:', idx)
    print(content[max(0,idx-50):idx+200])
    print('---')
    idx = content.find('QuantumCoreSupportSerializationTests', idx+1)