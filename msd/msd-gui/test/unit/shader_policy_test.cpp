// Ticket: 0002_remove_rotation_from_gpu
// Design: docs/designs/modularize-gpu-shader-system/design.md

#include <gtest/gtest.h>

#include <cmath>
#include <unordered_map>

#include <msd-gui/src/ShaderPolicy.hpp>

using namespace msd_gui;

//=============================================================================
// PositionOnlyShaderPolicy Tests
//=============================================================================

TEST(PositionOnlyShaderPolicy, VertexAttributeConfiguration)
{
  PositionOnlyShaderPolicy policy;
  auto attrs = policy.getVertexAttributes();

  ASSERT_EQ(attrs.size(), 5);

  // Per-vertex attributes (buffer slot 0)
  EXPECT_EQ(attrs[0].location, 0);                                      // Position
  EXPECT_EQ(attrs[0].buffer_slot, 0);
  EXPECT_EQ(attrs[0].format, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3);
  EXPECT_EQ(attrs[0].offset, 0);

  EXPECT_EQ(attrs[1].location, 1);                                      // Color
  EXPECT_EQ(attrs[1].buffer_slot, 0);
  EXPECT_EQ(attrs[1].format, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3);
  EXPECT_EQ(attrs[1].offset, sizeof(float) * 3);

  EXPECT_EQ(attrs[2].location, 2);                                      // Normal
  EXPECT_EQ(attrs[2].buffer_slot, 0);
  EXPECT_EQ(attrs[2].format, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3);
  EXPECT_EQ(attrs[2].offset, sizeof(float) * 6);

  // Per-instance attributes (buffer slot 1)
  EXPECT_EQ(attrs[3].location, 3);                                      // Instance position
  EXPECT_EQ(attrs[3].buffer_slot, 1);
  EXPECT_EQ(attrs[3].format, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3);
  EXPECT_EQ(attrs[3].offset, 0);

  EXPECT_EQ(attrs[4].location, 4);                                      // Instance color
  EXPECT_EQ(attrs[4].buffer_slot, 1);
  EXPECT_EQ(attrs[4].format, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3);
  EXPECT_EQ(attrs[4].offset, sizeof(float) * 3);
}

TEST(PositionOnlyShaderPolicy, VertexBufferDescriptions)
{
  PositionOnlyShaderPolicy policy;
  auto bufferDescs = policy.getVertexBufferDescriptions();

  ASSERT_EQ(bufferDescs.size(), 2);

  // Buffer slot 0: Per-vertex data
  EXPECT_EQ(bufferDescs[0].slot, 0);
  EXPECT_EQ(bufferDescs[0].pitch, sizeof(msd_assets::Vertex));
  EXPECT_EQ(bufferDescs[0].input_rate, SDL_GPU_VERTEXINPUTRATE_VERTEX);
  EXPECT_EQ(bufferDescs[0].instance_step_rate, 0);

  // Buffer slot 1: Per-instance data
  EXPECT_EQ(bufferDescs[1].slot, 1);
  EXPECT_EQ(bufferDescs[1].pitch, sizeof(PositionOnlyInstanceData));
  EXPECT_EQ(bufferDescs[1].input_rate, SDL_GPU_VERTEXINPUTRATE_INSTANCE);
  EXPECT_EQ(bufferDescs[1].instance_step_rate, 0);
}

TEST(PositionOnlyShaderPolicy, VertexInputState)
{
  PositionOnlyShaderPolicy policy;
  auto inputState = policy.getVertexInputState();

  EXPECT_EQ(inputState.num_vertex_buffers, 2);
  EXPECT_EQ(inputState.num_vertex_attributes, 5);
  EXPECT_NE(inputState.vertex_buffer_descriptions, nullptr);
  EXPECT_NE(inputState.vertex_attributes, nullptr);
}

TEST(PositionOnlyShaderPolicy, ShaderFileNames)
{
  PositionOnlyShaderPolicy policy;

  EXPECT_EQ(policy.getVertexShaderFile(), "Position3DColorTransform.vert");
  EXPECT_EQ(policy.getFragmentShaderFile(), "SolidColor.frag");
  EXPECT_EQ(policy.getInstanceDataSize(), 32);
}

//=============================================================================
// FullTransformShaderPolicy Tests
//=============================================================================

TEST(FullTransformShaderPolicy, VertexAttributeConfiguration)
{
  FullTransformShaderPolicy policy;
  auto attrs = policy.getVertexAttributes();

  ASSERT_EQ(attrs.size(), 9);

  // Per-vertex attributes (buffer slot 0)
  EXPECT_EQ(attrs[0].location, 0);                                      // Position
  EXPECT_EQ(attrs[0].buffer_slot, 0);
  EXPECT_EQ(attrs[0].format, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3);

  EXPECT_EQ(attrs[1].location, 1);                                      // Color
  EXPECT_EQ(attrs[1].buffer_slot, 0);

  EXPECT_EQ(attrs[2].location, 2);                                      // Normal
  EXPECT_EQ(attrs[2].buffer_slot, 0);

  // Per-instance model matrix (locations 3-6)
  EXPECT_EQ(attrs[3].location, 3);                                      // Model matrix row 0
  EXPECT_EQ(attrs[3].buffer_slot, 1);
  EXPECT_EQ(attrs[3].format, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4);
  EXPECT_EQ(attrs[3].offset, 0);

  EXPECT_EQ(attrs[4].location, 4);                                      // Model matrix row 1
  EXPECT_EQ(attrs[4].offset, sizeof(float) * 4);

  EXPECT_EQ(attrs[5].location, 5);                                      // Model matrix row 2
  EXPECT_EQ(attrs[5].offset, sizeof(float) * 8);

  EXPECT_EQ(attrs[6].location, 6);                                      // Model matrix row 3
  EXPECT_EQ(attrs[6].offset, sizeof(float) * 12);

  EXPECT_EQ(attrs[7].location, 7);                                      // Instance color
  EXPECT_EQ(attrs[7].buffer_slot, 1);
  EXPECT_EQ(attrs[7].format, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3);
  EXPECT_EQ(attrs[7].offset, sizeof(float) * 16);

  EXPECT_EQ(attrs[8].location, 8);                                      // Geometry index
  EXPECT_EQ(attrs[8].buffer_slot, 1);
  EXPECT_EQ(attrs[8].format, SDL_GPU_VERTEXELEMENTFORMAT_UINT);
  EXPECT_EQ(attrs[8].offset, sizeof(float) * 19);
}

TEST(FullTransformShaderPolicy, VertexBufferDescriptions)
{
  FullTransformShaderPolicy policy;
  auto bufferDescs = policy.getVertexBufferDescriptions();

  ASSERT_EQ(bufferDescs.size(), 2);

  // Buffer slot 0: Per-vertex data
  EXPECT_EQ(bufferDescs[0].slot, 0);
  EXPECT_EQ(bufferDescs[0].pitch, sizeof(msd_assets::Vertex));

  // Buffer slot 1: Per-instance data
  EXPECT_EQ(bufferDescs[1].slot, 1);
  EXPECT_EQ(bufferDescs[1].pitch, sizeof(FullTransformInstanceData));
  EXPECT_EQ(bufferDescs[1].input_rate, SDL_GPU_VERTEXINPUTRATE_INSTANCE);
}

TEST(FullTransformShaderPolicy, ShaderFileNames)
{
  FullTransformShaderPolicy policy;

  EXPECT_EQ(policy.getVertexShaderFile(), "PositionRotation3DColorTransform.vert");
  EXPECT_EQ(policy.getFragmentShaderFile(), "SolidColor.frag");
  EXPECT_EQ(policy.getInstanceDataSize(), 96);
}

//=============================================================================
// Instance Data Structure Tests
//=============================================================================

TEST(PositionOnlyInstanceData, SizeAndAlignment)
{
  // Verify that PositionOnlyInstanceData is 32 bytes (as designed)
  EXPECT_EQ(sizeof(PositionOnlyInstanceData), 32);

  // Verify 16-byte alignment
  EXPECT_EQ(sizeof(PositionOnlyInstanceData) % 16, 0);
}

TEST(FullTransformInstanceData, SizeAndAlignment)
{
  // Verify that FullTransformInstanceData is 96 bytes (as designed)
  EXPECT_EQ(sizeof(FullTransformInstanceData), 96);

  // Verify 16-byte alignment
  EXPECT_EQ(sizeof(FullTransformInstanceData) % 16, 0);
}
