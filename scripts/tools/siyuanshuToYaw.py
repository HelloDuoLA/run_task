from tf.transformations import quaternion_from_euler, euler_from_quaternion
x = 0
y = 0
z = -0.710
w = 0.703

result = euler_from_quaternion([x, y, z, w])
print(result)