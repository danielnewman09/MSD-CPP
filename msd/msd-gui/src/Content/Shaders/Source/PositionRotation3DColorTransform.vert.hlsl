// Ticket: 0001_link-gui-sim-object
// Design: docs/designs/generalize-gui-object-rendering/design.md

cbuffer TransformUBO : register(b0, space1)
{
    float4x4 viewProjection : packoffset(c0);  // View-Projection matrix (model is per-instance)
};

struct Input
{
    float3 Position : TEXCOORD0;  // Per-vertex data
    float3 Color : TEXCOORD1;     // Per-vertex data (unused, kept for compatibility)
    float3 Normal : TEXCOORD2;    // Per-vertex data

    // Per-instance model matrix (4x4 matrix = 4 vec4s)
    float4 InstanceModelRow0 : TEXCOORD3;
    float4 InstanceModelRow1 : TEXCOORD4;
    float4 InstanceModelRow2 : TEXCOORD5;
    float4 InstanceModelRow3 : TEXCOORD6;

    float3 InstanceColor : TEXCOORD7;      // Per-instance color
    uint InstanceGeometryIndex : TEXCOORD8;  // Per-instance geometry index (unused in shader for now)
};

struct Output
{
    float4 Color : TEXCOORD0;
    float3 Normal : TEXCOORD1;
    float3 FragPos : TEXCOORD2;
    float4 Position : SV_Position;
};

Output main(Input input)
{
    Output output;

    // Build model matrix from instance data (row-major)
    float4x4 instanceModel = transpose(float4x4(
        input.InstanceModelRow0,
        input.InstanceModelRow1,
        input.InstanceModelRow2,
        input.InstanceModelRow3
    ));

    // Transform vertex position to world space using instance model matrix
    float4 worldPos = mul(instanceModel, float4(input.Position, 1.0f));
    output.FragPos = worldPos.xyz;

    // Transform world position to clip space using view-projection matrix
    output.Position = mul(viewProjection, worldPos);

    // Transform normal to world space (use upper-left 3x3 of model matrix)
    float3x3 normalMatrix = float3x3(
        instanceModel[0].xyz,
        instanceModel[1].xyz,
        instanceModel[2].xyz
    );
    output.Normal = mul(normalMatrix, input.Normal);

    // Use instance color
    output.Color = float4(input.InstanceColor, 1.0f);

    return output;
}
