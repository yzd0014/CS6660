#version 150
// in_Position was bound to attribute index 0 and in_Color was bound to attribute index 1
in  vec3 in_Position;
in  vec3 in_Color;

// We output the ex_Color variable to the next shader in the chain
out vec3 ex_Color;
uniform mat4 MVP;

void main(void) {
    // Since we are using flat lines, our input only had two points: x and y.
    // Set the Z coordinate to 0 and W coordinate to 1

    gl_Position = MVP*vec4(in_Position.x, in_Position.y, in_Position.z, 1.0);

    // GLSL allows shorthand use of vectors too, the following is also valid:
    // gl_Position = vec4(in_Position, 0.0, 1.0);

    // We're simply passing the color through unmodified
    ex_Color = in_Color;
}
