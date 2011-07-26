Jello Cube physics simulation
-Rejah Anuvar
=============
To Build/Exec:
$ make
$ ./jello jello.w

Info:
- Simulates a jello Cube using a 8 * 8 * 8 mass spring system.
- The program reads the jello and the bounding box co-ordinates from the jello.w file.
- The jello has structural, shear and bend springs which can be seen by toggling 's', 'h', 'b' keys.
- There is an external force field which affects the control points and the forces are interpolated on the points
- Checks for collision with the bounding box and computes response bounces.
- Collission forces calculated based on penetration hence soft bounces.
- Used Goroud shading for shading.

Controls/ Input:
ESC: exit application
v  : switch wireframe /triangle mode
s  : display structural springs on/off
h  : display shear springs on/off
b  : display bend springs on/off
space : save the current screen to a file, filename index increments automatically
p  : pause on/off
z  : camera zoom in
x  : camera zoom out
right mouse button + move mouse: camera control
e  : reset camera to default position
