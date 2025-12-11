struct Input
{
    float4 Color : TEXCOORD0;
    float3 Normal : TEXCOORD1;
    float3 FragPos : TEXCOORD2;
};

float4 main(Input input) : SV_Target0
{
    // Simple directional light (coming from upper-left-front)
    float3 lightDir = normalize(float3(-0.3, -0.5, -0.8));

    // Ambient lighting (base color so nothing is completely black)
    float ambientStrength = 0.3;
    float3 ambient = ambientStrength * input.Color.rgb;

    // Diffuse lighting
    float3 norm = normalize(input.Normal);
    float diff = max(dot(norm, -lightDir), 0.0);
    float3 diffuse = diff * input.Color.rgb;

    // Combine ambient and diffuse
    float3 result = ambient + diffuse;

    return float4(result, input.Color.a);
}
