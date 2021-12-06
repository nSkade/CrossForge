#include <CForge/Graphics/Shader/SShaderManager.h>
#include <CForge/Graphics/STextureManager.h>
#include <glad/glad.h>
#include "HeightMap.h"

using namespace std;

float randf() {
    return static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
}

namespace Terrain {
    HeightMap::HeightMap() : mTexture() {
        initShader();
    }

    void HeightMap::generate(HeightMapConfig config) {
        GLint internalFormat = GL_R16UI;
        GLint format = GL_RED_INTEGER;
        GLint dataType = GL_UNSIGNED_SHORT;

        GLuint textureHandle;
        glGenTextures(1, &textureHandle);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, textureHandle);
        // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
        float borderColor[] = { 0.0f, 0.0f, 0.0f, 0.0f };
        glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, config.width, config.height, 0, format, dataType, NULL);
        glBindImageTexture(0, textureHandle, 0, GL_FALSE, 0, GL_WRITE_ONLY, internalFormat);

        mTexture = STextureManager::fromHandle(textureHandle);

        mShader->bind();
        glActiveTexture(GL_TEXTURE0);
        glBindImageTexture(0, mTexture->handle(), 0, GL_FALSE, 0, GL_WRITE_ONLY, format);
        bindNoiseData(config.noiseConfig);
        glDispatchCompute(config.width, config.height, 1);
    }

    void HeightMap::setTexture(GLTexture2D* texture) {
        mTexture = texture;
    }

    void HeightMap::bindTexture() {
        mTexture->bind();
    }

    void HeightMap::bindNoiseData(NoiseConfig config) {
        srand(config.seed);

        glUniform1f(mShader->uniformLocation("Noise.scale"), config.scale);
        glUniform1ui(mShader->uniformLocation("Noise.octaves"), config.octaves);
        glUniform1f(mShader->uniformLocation("Noise.persistence"), config.persistence);
        glUniform1f(mShader->uniformLocation("Noise.lacunarity"), config.lacunarity);

        for (int i = 0; i < config.octaves; ++i) {
            glUniform2f(mShader->uniformLocation("Noise.offsets[" + to_string(i) + "]"), randf(), randf());
        }
    }

    void HeightMap::initShader() {
        vector<ShaderCode*> csSources;
        string errorLog;

        SShaderManager* shaderManager = SShaderManager::instance();
        csSources.push_back(shaderManager->createShaderCode("Shader/HeightMapShader.comp", "430",
                                                            0, "", ""));
        mShader = shaderManager->buildComputeShader(&csSources, &errorLog);

        shaderManager->release();
    }
}