#include "gpu_outline.h"

#include <cstddef>
#include <cstring>

#include "openvr.h"

#ifndef GPU_OUTLINE_SHADER_DIR
#define GPU_OUTLINE_SHADER_DIR "shaders"
#endif

// clang-format off
#define GPU_OUTLINE_EMBED(symbol, file)                            \
  __asm__(".pushsection .rodata\n"                                 \
          ".balign 16\n"                                           \
          ".global " #symbol "\n"                                  \
          #symbol ":\n"                                            \
          ".incbin \"" GPU_OUTLINE_SHADER_DIR "/" file "\"\n"      \
          ".global " #symbol "_end\n"                              \
          #symbol "_end:\n"                                        \
          ".byte 0\n"                                              \
          ".popsection\n");                                        \
  extern "C" {                                                     \
  extern const unsigned char symbol[];                             \
  extern const unsigned char symbol##_end[];                       \
  }
// clang-format on

GPU_OUTLINE_EMBED(gpu_outline_vert_spv, "outline.vert.spv")
GPU_OUTLINE_EMBED(gpu_outline_frag_spv, "outline.frag.spv")

namespace {

constexpr VkFormat kFormat = VK_FORMAT_R8G8B8A8_UNORM;

std::string vk_error(const char* what, VkResult result) {
  return std::string(what) + " failed (VkResult " + std::to_string(int(result)) + ")";
}

VkShaderModule shader_module(VkDevice device, const unsigned char* begin, const unsigned char* end) {
  VkShaderModuleCreateInfo info{};
  info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
  info.codeSize = size_t(end - begin);
  info.pCode = reinterpret_cast<const uint32_t*>(begin);
  VkShaderModule module = VK_NULL_HANDLE;
  return vkCreateShaderModule(device, &info, nullptr, &module) == VK_SUCCESS ? module : VK_NULL_HANDLE;
}

}  // namespace

GpuOutline::~GpuOutline() { destroy(); }

bool GpuOutline::create(VulkanContext& context, int size, std::string& error) {
  destroy();
  context_ = &context;
  size_ = size;
  const VkDevice device = context.device();
  const auto failed = [&](const char* what, VkResult result) {
    error = vk_error(what, result);
    destroy();
    return false;
  };
  VkResult result = VK_SUCCESS;

  VkAttachmentDescription attachment{};
  attachment.format = kFormat;
  attachment.samples = VK_SAMPLE_COUNT_1_BIT;
  attachment.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
  attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
  attachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
  attachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  attachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  attachment.finalLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
  const VkAttachmentReference colour{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
  VkSubpassDescription subpass{};
  subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
  subpass.colorAttachmentCount = 1;
  subpass.pColorAttachments = &colour;
  VkSubpassDependency dependencies[2] = {};
  dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
  dependencies[0].dstSubpass = 0;
  dependencies[0].srcStageMask = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT;
  dependencies[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  dependencies[0].srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT | VK_ACCESS_SHADER_READ_BIT;
  dependencies[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
  dependencies[1].srcSubpass = 0;
  dependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
  dependencies[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  dependencies[1].dstStageMask = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT;
  dependencies[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
  dependencies[1].dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT | VK_ACCESS_SHADER_READ_BIT;
  VkRenderPassCreateInfo pass_info{};
  pass_info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
  pass_info.attachmentCount = 1;
  pass_info.pAttachments = &attachment;
  pass_info.subpassCount = 1;
  pass_info.pSubpasses = &subpass;
  pass_info.dependencyCount = 2;
  pass_info.pDependencies = dependencies;
  result = vkCreateRenderPass(device, &pass_info, nullptr, &render_pass_);
  if (result != VK_SUCCESS) return failed("vkCreateRenderPass", result);

  VkBufferCreateInfo buffer_info{};
  buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_info.size = sizeof(OutlineUniforms);
  buffer_info.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
  buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  result = vkCreateBuffer(device, &buffer_info, nullptr, &uniforms_);
  if (result != VK_SUCCESS) return failed("vkCreateBuffer", result);
  VkMemoryRequirements need{};
  vkGetBufferMemoryRequirements(device, uniforms_, &need);
  VkMemoryAllocateInfo allocate{};
  allocate.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  allocate.allocationSize = need.size;
  allocate.memoryTypeIndex = context.findMemoryType(
      need.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
  result = allocate.memoryTypeIndex == UINT32_MAX ? VK_ERROR_FEATURE_NOT_PRESENT
                                                  : vkAllocateMemory(device, &allocate, nullptr, &uniforms_memory_);
  if (result != VK_SUCCESS) return failed("vkAllocateMemory(uniforms)", result);
  vkBindBufferMemory(device, uniforms_, uniforms_memory_, 0);
  result = vkMapMemory(device, uniforms_memory_, 0, sizeof(OutlineUniforms), 0, &uniforms_mapped_);
  if (result != VK_SUCCESS) return failed("vkMapMemory", result);

  VkDescriptorSetLayoutBinding binding{};
  binding.binding = 0;
  binding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
  binding.descriptorCount = 1;
  binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
  VkDescriptorSetLayoutCreateInfo set_info{};
  set_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  set_info.bindingCount = 1;
  set_info.pBindings = &binding;
  result = vkCreateDescriptorSetLayout(device, &set_info, nullptr, &set_layout_);
  if (result != VK_SUCCESS) return failed("vkCreateDescriptorSetLayout", result);
  const VkDescriptorPoolSize pool_size{VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1};
  VkDescriptorPoolCreateInfo pool_info{};
  pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
  pool_info.maxSets = 1;
  pool_info.poolSizeCount = 1;
  pool_info.pPoolSizes = &pool_size;
  result = vkCreateDescriptorPool(device, &pool_info, nullptr, &descriptor_pool_);
  if (result != VK_SUCCESS) return failed("vkCreateDescriptorPool", result);
  VkDescriptorSetAllocateInfo set_allocate{};
  set_allocate.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
  set_allocate.descriptorPool = descriptor_pool_;
  set_allocate.descriptorSetCount = 1;
  set_allocate.pSetLayouts = &set_layout_;
  result = vkAllocateDescriptorSets(device, &set_allocate, &descriptor_set_);
  if (result != VK_SUCCESS) return failed("vkAllocateDescriptorSets", result);
  const VkDescriptorBufferInfo whole{uniforms_, 0, sizeof(OutlineUniforms)};
  VkWriteDescriptorSet write{};
  write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
  write.dstSet = descriptor_set_;
  write.dstBinding = 0;
  write.descriptorCount = 1;
  write.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
  write.pBufferInfo = &whole;
  vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);

  VkPipelineLayoutCreateInfo layout_info{};
  layout_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
  layout_info.setLayoutCount = 1;
  layout_info.pSetLayouts = &set_layout_;
  result = vkCreatePipelineLayout(device, &layout_info, nullptr, &pipeline_layout_);
  if (result != VK_SUCCESS) return failed("vkCreatePipelineLayout", result);

  const VkShaderModule vert = shader_module(device, gpu_outline_vert_spv, gpu_outline_vert_spv_end);
  const VkShaderModule frag = shader_module(device, gpu_outline_frag_spv, gpu_outline_frag_spv_end);
  result = VK_ERROR_INVALID_SHADER_NV;
  if (vert != VK_NULL_HANDLE && frag != VK_NULL_HANDLE) {
    VkPipelineShaderStageCreateInfo stages[2] = {};
    stages[0].sType = stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
    stages[0].module = vert;
    stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    stages[1].module = frag;
    stages[0].pName = stages[1].pName = "main";
    VkPipelineVertexInputStateCreateInfo vertex_input{};
    vertex_input.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    VkPipelineInputAssemblyStateCreateInfo assembly{};
    assembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    assembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    const VkViewport viewport{0, 0, float(size), float(size), 0, 1};
    const VkRect2D scissor{{0, 0}, {uint32_t(size), uint32_t(size)}};
    VkPipelineViewportStateCreateInfo viewports{};
    viewports.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
    viewports.viewportCount = 1;
    viewports.pViewports = &viewport;
    viewports.scissorCount = 1;
    viewports.pScissors = &scissor;
    VkPipelineRasterizationStateCreateInfo raster{};
    raster.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    raster.polygonMode = VK_POLYGON_MODE_FILL;
    raster.cullMode = VK_CULL_MODE_NONE;
    raster.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    raster.lineWidth = 1;
    VkPipelineMultisampleStateCreateInfo multisample{};
    multisample.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
    multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    VkPipelineColorBlendAttachmentState blend_attachment{};
    blend_attachment.colorWriteMask =
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    VkPipelineColorBlendStateCreateInfo blend{};
    blend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    blend.attachmentCount = 1;
    blend.pAttachments = &blend_attachment;
    VkGraphicsPipelineCreateInfo pipeline_info{};
    pipeline_info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
    pipeline_info.stageCount = 2;
    pipeline_info.pStages = stages;
    pipeline_info.pVertexInputState = &vertex_input;
    pipeline_info.pInputAssemblyState = &assembly;
    pipeline_info.pViewportState = &viewports;
    pipeline_info.pRasterizationState = &raster;
    pipeline_info.pMultisampleState = &multisample;
    pipeline_info.pColorBlendState = &blend;
    pipeline_info.layout = pipeline_layout_;
    pipeline_info.renderPass = render_pass_;
    result = vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &pipeline_info, nullptr, &pipeline_);
    if (result != VK_SUCCESS) pipeline_ = VK_NULL_HANDLE;
  }
  if (vert != VK_NULL_HANDLE) vkDestroyShaderModule(device, vert, nullptr);
  if (frag != VK_NULL_HANDLE) vkDestroyShaderModule(device, frag, nullptr);
  if (result != VK_SUCCESS) return failed("creating the outline's pipeline", result);

  for (int hand = 0; hand < 2; ++hand) {
    for (int i = 0; i < 2; ++i) {
      VkImageCreateInfo image_info{};
      image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
      image_info.imageType = VK_IMAGE_TYPE_2D;
      image_info.format = kFormat;
      image_info.extent = {uint32_t(size), uint32_t(size), 1};
      image_info.mipLevels = 1;
      image_info.arrayLayers = 1;
      image_info.samples = VK_SAMPLE_COUNT_1_BIT;
      image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
      image_info.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT |
                         VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
      image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
      image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
      result = vkCreateImage(device, &image_info, nullptr, &images_[hand][i]);
      if (result != VK_SUCCESS) return failed("vkCreateImage", result);
      vkGetImageMemoryRequirements(device, images_[hand][i], &need);
      allocate.allocationSize = need.size;
      allocate.memoryTypeIndex = context.findMemoryType(need.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
      result = allocate.memoryTypeIndex == UINT32_MAX
                   ? VK_ERROR_FEATURE_NOT_PRESENT
                   : vkAllocateMemory(device, &allocate, nullptr, &image_memory_[hand][i]);
      if (result != VK_SUCCESS) return failed("vkAllocateMemory(image)", result);
      vkBindImageMemory(device, images_[hand][i], image_memory_[hand][i], 0);
      VkImageViewCreateInfo view_info{};
      view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
      view_info.image = images_[hand][i];
      view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
      view_info.format = kFormat;
      view_info.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
      result = vkCreateImageView(device, &view_info, nullptr, &views_[hand][i]);
      if (result != VK_SUCCESS) return failed("vkCreateImageView", result);
      VkFramebufferCreateInfo framebuffer_info{};
      framebuffer_info.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
      framebuffer_info.renderPass = render_pass_;
      framebuffer_info.attachmentCount = 1;
      framebuffer_info.pAttachments = &views_[hand][i];
      framebuffer_info.width = framebuffer_info.height = uint32_t(size);
      framebuffer_info.layers = 1;
      result = vkCreateFramebuffer(device, &framebuffer_info, nullptr, &framebuffers_[hand][i]);
      if (result != VK_SUCCESS) return failed("vkCreateFramebuffer", result);
    }
  }

  VkCommandBufferAllocateInfo command_info{};
  command_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
  command_info.commandPool = context.commandPool();
  command_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
  command_info.commandBufferCount = 1;
  result = vkAllocateCommandBuffers(device, &command_info, &commands_);
  if (result != VK_SUCCESS) return failed("vkAllocateCommandBuffers", result);
  VkFenceCreateInfo fence_info{};
  fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
  result = vkCreateFence(device, &fence_info, nullptr, &fence_);
  if (result != VK_SUCCESS) return failed("vkCreateFence", result);
  return true;
}

void GpuOutline::destroy() {
  if (context_ != nullptr && context_->ready()) {
    const VkDevice device = context_->device();
    vkDeviceWaitIdle(device);
    if (fence_ != VK_NULL_HANDLE) vkDestroyFence(device, fence_, nullptr);
    if (commands_ != VK_NULL_HANDLE) vkFreeCommandBuffers(device, context_->commandPool(), 1, &commands_);
    for (int hand = 0; hand < 2; ++hand) {
      for (int i = 0; i < 2; ++i) {
        if (framebuffers_[hand][i] != VK_NULL_HANDLE) vkDestroyFramebuffer(device, framebuffers_[hand][i], nullptr);
        if (views_[hand][i] != VK_NULL_HANDLE) vkDestroyImageView(device, views_[hand][i], nullptr);
        if (images_[hand][i] != VK_NULL_HANDLE) vkDestroyImage(device, images_[hand][i], nullptr);
        if (image_memory_[hand][i] != VK_NULL_HANDLE) vkFreeMemory(device, image_memory_[hand][i], nullptr);
      }
    }
    if (pipeline_ != VK_NULL_HANDLE) vkDestroyPipeline(device, pipeline_, nullptr);
    if (pipeline_layout_ != VK_NULL_HANDLE) vkDestroyPipelineLayout(device, pipeline_layout_, nullptr);
    if (descriptor_pool_ != VK_NULL_HANDLE) vkDestroyDescriptorPool(device, descriptor_pool_, nullptr);
    if (set_layout_ != VK_NULL_HANDLE) vkDestroyDescriptorSetLayout(device, set_layout_, nullptr);
    if (uniforms_ != VK_NULL_HANDLE) vkDestroyBuffer(device, uniforms_, nullptr);
    if (uniforms_memory_ != VK_NULL_HANDLE) vkFreeMemory(device, uniforms_memory_, nullptr);
    if (render_pass_ != VK_NULL_HANDLE) vkDestroyRenderPass(device, render_pass_, nullptr);
  }
  for (int hand = 0; hand < 2; ++hand) {
    for (int i = 0; i < 2; ++i) {
      framebuffers_[hand][i] = VK_NULL_HANDLE;
      views_[hand][i] = VK_NULL_HANDLE;
      images_[hand][i] = VK_NULL_HANDLE;
      image_memory_[hand][i] = VK_NULL_HANDLE;
    }
    next_[hand] = 0;
  }
  fence_ = VK_NULL_HANDLE;
  commands_ = VK_NULL_HANDLE;
  pipeline_ = VK_NULL_HANDLE;
  pipeline_layout_ = VK_NULL_HANDLE;
  descriptor_pool_ = VK_NULL_HANDLE;
  descriptor_set_ = VK_NULL_HANDLE;
  set_layout_ = VK_NULL_HANDLE;
  uniforms_ = VK_NULL_HANDLE;
  uniforms_memory_ = VK_NULL_HANDLE;
  uniforms_mapped_ = nullptr;
  render_pass_ = VK_NULL_HANDLE;
  context_ = nullptr;
  size_ = 0;
}

bool GpuOutline::paint(int hand, uint64_t overlay, const OutlineLayout& layout, uint8_t flags, bool mask, uint8_t hold,
                       std::string& error) {
  if (!ready() || fence_ == VK_NULL_HANDLE || hand < 0 || hand > 1 || layout.size != size_) {
    error = "the GPU outline is not set up for this image";
    return false;
  }
  const VkDevice device = context_->device();
  const int index = next_[hand];
  next_[hand] = 1 - index;
  const OutlineUniforms uniforms = outline_uniforms(layout, flags, mask, hold);
  std::memcpy(uniforms_mapped_, &uniforms, sizeof uniforms);

  vkResetCommandBuffer(commands_, 0);
  VkCommandBufferBeginInfo begin{};
  begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
  vkBeginCommandBuffer(commands_, &begin);
  VkRenderPassBeginInfo pass{};
  pass.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
  pass.renderPass = render_pass_;
  pass.framebuffer = framebuffers_[hand][index];
  pass.renderArea = {{0, 0}, {uint32_t(size_), uint32_t(size_)}};
  vkCmdBeginRenderPass(commands_, &pass, VK_SUBPASS_CONTENTS_INLINE);
  vkCmdBindPipeline(commands_, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_);
  vkCmdBindDescriptorSets(commands_, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_layout_, 0, 1, &descriptor_set_, 0,
                          nullptr);
  vkCmdDraw(commands_, 3, 1, 0, 0);
  vkCmdEndRenderPass(commands_);
  vkEndCommandBuffer(commands_);

  VkSubmitInfo submit{};
  submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  submit.commandBufferCount = 1;
  submit.pCommandBuffers = &commands_;
  vkResetFences(device, 1, &fence_);
  VkResult result = vkQueueSubmit(context_->queue(), 1, &submit, fence_);
  if (result != VK_SUCCESS) {
    error = vk_error("vkQueueSubmit", result);
    return false;
  }
  result = vkWaitForFences(device, 1, &fence_, VK_TRUE, 1'000'000'000ull);
  if (result != VK_SUCCESS) {
    error = vk_error("vkWaitForFences", result);
    return false;
  }

  vr::VRVulkanTextureData_t data{};
  data.m_nImage = reinterpret_cast<uint64_t>(images_[hand][index]);
  data.m_pDevice = reinterpret_cast<VkDevice_T*>(device);
  data.m_pPhysicalDevice = reinterpret_cast<VkPhysicalDevice_T*>(context_->physicalDevice());
  data.m_pInstance = reinterpret_cast<VkInstance_T*>(context_->instance());
  data.m_pQueue = reinterpret_cast<VkQueue_T*>(context_->queue());
  data.m_nQueueFamilyIndex = context_->queueFamily();
  data.m_nWidth = data.m_nHeight = uint32_t(size_);
  data.m_nFormat = kFormat;
  data.m_nSampleCount = 1;
  vr::Texture_t texture{&data, vr::TextureType_Vulkan, vr::ColorSpace_Gamma};
  const vr::EVROverlayError overlay_error = vr::VROverlay()->SetOverlayTexture(overlay, &texture);
  if (overlay_error != vr::VROverlayError_None) {
    error = std::string("SetOverlayTexture: ") + vr::VROverlay()->GetOverlayErrorNameFromEnum(overlay_error);
    return false;
  }
  return true;
}
