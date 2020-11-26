
Texture2D txDiffuse;
SamplerState samLinear;

struct VOut
{
    float4 position : SV_POSITION;
	float2 tex : TEXCOORD;
};

VOut VShader(float4 position : POSITION, float2 tex : TEXCOORD)
{
    VOut output;

    output.position = position;
    output.tex = tex;

    return output;
}


float4 PShader(float4 position : SV_POSITION, float4 color : COLOR) : SV_TARGET
{
    return color;
}
