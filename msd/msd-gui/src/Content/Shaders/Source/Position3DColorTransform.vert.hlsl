cbuffer TransformUBO : register(b0, space1)
{
    float4x4 modelViewProjection : packoffset(c0);
};

struct Input
{
    float3 Position : TEXCOORD0;  // Per-vertex data
    float3 Color : TEXCOORD1;     // Per-vertex data (will be overridden by instance color)
    float3 Normal : TEXCOORD2;    // Per-vertex data
    float3 InstancePosition : TEXCOORD3;  // Per-instance data
    float3 InstanceColor : TEXCOORD4;     // Per-instance data
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
    // Use instance color instead of vertex color
    output.Color = float4(input.InstanceColor, 1.0f);
    output.Normal = input.Normal;

    // Apply instance position offset to vertex position
    float3 worldPos = input.Position + input.InstancePosition;
    output.FragPos = worldPos;

    // Apply model-view-projection transformation for 3D
    output.Position = mul(modelViewProjection, float4(worldPos, 1.0f));
    return output;
}
