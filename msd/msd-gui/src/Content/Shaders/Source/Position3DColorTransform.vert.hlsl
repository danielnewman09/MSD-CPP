cbuffer TransformUBO : register(b0, space1)
{
    float4x4 modelViewProjection : packoffset(c0);
};

struct Input
{
    float3 Position : TEXCOORD0;
    float3 Color : TEXCOORD1;
};

struct Output
{
    float4 Color : TEXCOORD0;
    float4 Position : SV_Position;
};

Output main(Input input)
{
    Output output;
    output.Color = float4(input.Color, 1.0f);
    // Apply model-view-projection transformation for 3D
    output.Position = mul(modelViewProjection, float4(input.Position, 1.0f));
    return output;
}
