#pragma once

class EskyWindow {
 public:
  virtual ~EskyWindow() {}

  void setTitle(const wchar_t*);
  void setRect(int, int, int, int);

  virtual void setDeltas(void*, void*) = 0;
  virtual void setRequiredValues(...) = 0;
  virtual void setRenderTextureWidthHeight(int, int) = 0;
  virtual void setEnableFlagWarping(bool) = 0;
  virtual void setBrightness(float) = 0;
  virtual void sendTextureIdToPluginByIdLeft(void*) = 0;
  virtual void sendTextureIdToPluginByIdRight(void*) = 0;

 protected:
  virtual void _onResize() = 0;
};
