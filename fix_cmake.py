f = open('tests/CMakeLists.txt', 'r')
content = f.read()
f.close()

# Find the second occurrence of the target_link_libraries block for SupportSerialization
# We need to remove the duplicate block starting at position 42045
# The first occurrence ends around 40615, and the second starts around 42045

# Find the first occurrence of the second block
idx1 = content.find('target_link_libraries(\n    QuantumCoreSupportSerializationTests', 41000)
print('Second target_link_libraries at:', idx1)

# Find the end of the second block (after set_tests_properties)
idx2 = content.find('if(QUANTUM_ENABLE_EDITOR)', idx1)
print('End of second block at:', idx2)

# Remove the duplicate block
if idx1 >= 0 and idx2 >= 0:
    new_content = content[:idx1] + content[idx2:]
    f = open('tests/CMakeLists.txt', 'w')
    f.write(new_content)
    f.close()
    print('Fixed!')
else:
    print('Could not find boundaries')