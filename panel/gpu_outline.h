#pragma once

#include <vulkan/vulkan.h>

#include <cstdint>
#include <string>

#include "hand_outline.h"
#include "vk_texture.h"

class GpuOutline {
 public:
  GpuOutline() = default;
  ~GpuOutline();
  GpuOutline(const GpuOutline&) = delete;
  GpuOutline& operator=(const GpuOutline&) = delete;

  bool create(VulkanContext& context, int size, std::string& error);
  void destroy();
  bool ready() const { return pipeline_ != VK_NULL_HANDLE; }
  int size() const { return size_; }

  bool paint(int hand, uint64_t overlay, const OutlineLayout& layout, uint8_t flags, bool mask, uint8_t hold,
             std::string& error);

 private:
  VulkanContext* context_ = nullptr;
  int size_ = 0;
  VkRenderPass render_pass_ = VK_NULL_HANDLE;
  VkDescriptorSetLayout set_layout_ = VK_NULL_HANDLE;
  VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
  VkPipeline pipeline_ = VK_NULL_HANDLE;
  VkDescriptorPool descriptor_pool_ = VK_NULL_HANDLE;
  VkDescriptorSet descriptor_set_ = VK_NULL_HANDLE;
  VkBuffer uniforms_ = VK_NULL_HANDLE;
  VkDeviceMemory uniforms_memory_ = VK_NULL_HANDLE;
  void* uniforms_mapped_ = nullptr;
  VkCommandBuffer commands_ = VK_NULL_HANDLE;
  VkFence fence_ = VK_NULL_HANDLE;
  VkImage images_[2][2] = {};
  VkDeviceMemory image_memory_[2][2] = {};
  VkImageView views_[2][2] = {};
  VkFramebuffer framebuffers_[2][2] = {};
  int next_[2] = {};
};
