#pragma once

#include <glad/glad.h>
#include <CForge/Graphics/Lights/DirectionalLight.h>
#include <CForge/Graphics/GraphicsUtility.h>
#include <CForge/Graphics/SceneGraph/SceneGraph.h>
#include <CForge/Graphics/Shader/SShaderManager.h>
#include <CForge/Graphics/STextureManager.h>

#include "Terrain/src/Map/TerrainMap.h"
#include "./Decoration/DecoSetup.hpp"

using namespace CForge;

namespace Terrain {

    void initCForge(GLWindow *window, RenderDevice *renderDevice, VirtualCamera *camera, DirectionalLight *sun,
                    DirectionalLight *light) {
        uint32_t winWidth = 720;
        uint32_t winHeight = 720;

        winWidth = 1200;
        winHeight = 1200;

        window->init(Vector2i(200, 200), Vector2i(winWidth, winHeight), "Terrain Setup");
        gladLoadGL();

        string GLError;
        GraphicsUtility::checkGLError(&GLError);
        if (!GLError.empty()) printf("GLError occurred: %s\n", GLError.c_str());

        RenderDevice::RenderDeviceConfig renderConfig;
        renderConfig.DirectionalLightsCount = 2;
        renderConfig.PointLightsCount = 0;
        renderConfig.SpotLightsCount = 0;
        renderConfig.ExecuteLightingPass = true;
        renderConfig.GBufferWidth = winWidth;
        renderConfig.GBufferHeight = winHeight;
        renderConfig.pAttachedWindow = window;
        renderConfig.PhysicallyBasedShading = true;
        renderConfig.UseGBuffer = true;
        renderDevice->init(&renderConfig);

        ShaderCode::LightConfig lightConfig;
        lightConfig.DirLightCount = 2;
        lightConfig.PCFSize = 1;
        lightConfig.PointLightCount = 0;
        lightConfig.ShadowBias = 0.0004f;
        lightConfig.ShadowMapCount = 1;
        lightConfig.SpotLightCount = 0;
        SShaderManager *shaderManager = SShaderManager::instance();
        shaderManager->configShader(lightConfig);
        shaderManager->release();

        camera->init(Vector3f(.0f, 400.0f, 100.0f), Vector3f::UnitY());
        camera->pitch(GraphicsUtility::degToRad(-15.0f));
        camera->projectionMatrix(winWidth, winHeight, GraphicsUtility::degToRad(45.0f), 1.0f, 100000.0f);
        renderDevice->activeCamera(camera);

        Vector3f sunPos = Vector3f(400.0f, 400.0f, 0.0f);
        sun->init(sunPos, -sunPos.normalized(), Vector3f(1.0f, 1.0f, 1.0f), 2.5f);
        auto projection = GraphicsUtility::perspectiveProjection(winWidth, winHeight, GraphicsUtility::degToRad(45.0f),
                                                                 1.0f, 10000.0f);
        const uint32_t ShadowMapDim = 4096;
        sun->initShadowCasting(ShadowMapDim, ShadowMapDim, Eigen::Vector2i(1250, 1250), 1000.0f, 100000.0f);
        renderDevice->addLight(sun);

        Vector3f lightPos = Vector3f(-2000.0f, 2000.0f, -1000.0f);
        light->init(lightPos, -lightPos.normalized(), Vector3f(1.0f, 1.0f, 1.0f), 0.5f);
        renderDevice->addLight(light);
    }

    void initDebugQuad(ScreenQuad *quad) {
        vector<ShaderCode *> vsSources;
        vector<ShaderCode *> fsSources;
        string errorLog;

        SShaderManager *shaderManager = SShaderManager::instance();
        vsSources.push_back(shaderManager->createShaderCode("Shader/ScreenQuad.vert", "330 core",
                                                            0, "", ""));
        fsSources.push_back(shaderManager->createShaderCode("Shader/DebugQuad.frag", "330 core",
                                                            0, "", ""));
        GLShader *quadShader = shaderManager->buildShader(&vsSources, &fsSources, &errorLog);
        shaderManager->release();

        quad->init(0, 0, 1, 1, quadShader);
    }

    void updateCamera(Mouse *mouse, Keyboard *keyboard, VirtualCamera *camera) {
        if (nullptr == keyboard) return;

        const float movementSpeed = 4;

        float MovementScale = 0.1f;
        if (keyboard->keyPressed(Keyboard::KEY_LEFT_SHIFT) || keyboard->keyPressed(Keyboard::KEY_RIGHT_SHIFT)) {
            MovementScale = 0.3f;
        }
        if (keyboard->keyPressed(Keyboard::KEY_Q)) {
            MovementScale = 0.02f;
        }

        if (keyboard->keyPressed(Keyboard::KEY_W) || keyboard->keyPressed(Keyboard::KEY_UP))
            camera->forward(movementSpeed * MovementScale);
        if (keyboard->keyPressed(Keyboard::KEY_S) || keyboard->keyPressed(Keyboard::KEY_DOWN))
            camera->forward(-movementSpeed * MovementScale);
        if (keyboard->keyPressed(Keyboard::KEY_A) || keyboard->keyPressed(Keyboard::KEY_LEFT))
            camera->right(-movementSpeed * MovementScale);
        if (keyboard->keyPressed(Keyboard::KEY_D) || keyboard->keyPressed(Keyboard::KEY_RIGHT))
            camera->right(movementSpeed * MovementScale);
        if (keyboard->keyPressed(Keyboard::KEY_LEFT_ALT)) camera->up(-movementSpeed * MovementScale);
        if (keyboard->keyPressed(Keyboard::KEY_LEFT_CONTROL)) camera->up(movementSpeed * MovementScale);

        // rotation
        if (mouse->buttonState(Mouse::BTN_RIGHT)) {
            Vector2f MouseDelta = mouse->movement();

            camera->rotY(GraphicsUtility::degToRad(MouseDelta.x()) * -0.1f * movementSpeed);
            camera->pitch(GraphicsUtility::degToRad(MouseDelta.y()) * -0.1f * movementSpeed);

            mouse->movement(Eigen::Vector2f::Zero());
        }
    }

    void setWindUBO(unsigned int windUBO, Vector3f& windVec, float time) {
        glBindBuffer(GL_UNIFORM_BUFFER, windUBO);
        glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(float) * 3, windVec.data());
        glBufferSubData(GL_UNIFORM_BUFFER, sizeof(float) * 3, sizeof(float), &time);
        glBindBuffer(GL_UNIFORM_BUFFER, 0);
    }

    float sign(Vector2f p1, Vector2f p2, Vector2f p3)
    {
        return (p1.x() - p3.x()) * (p2.y() - p3.y()) - (p2.x() - p3.x()) * (p1.y() - p3.y());
    }

    bool PointInTriangle(Vector2f pt, Vector2f v1, Vector2f v2, Vector2f v3)
    {
        float d1, d2, d3;
        bool has_neg, has_pos;

        d1 = sign(pt, v1, v2);
        d2 = sign(pt, v2, v3);
        d3 = sign(pt, v3, v1);

        has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

        return !(has_neg && has_pos);
    }

    void updateGrass(InstanceActor& iGrassActor, TerrainMap& map, VirtualCamera& camera) {
        iGrassActor.clearInstances();
        for (int i = 3; i > 0; i--) {
            float scale = exp2(i);
            float triangleHeight = 20 * scale;
            float triangleWidth = 30 * scale;
            float smallerTriangleHeight = (i == 1) ? 0.1 : triangleHeight / 2.0;
            float smallerTriangleWidth = (i == 1) ? 0.1 : triangleWidth / 2.0;

            float spacing = 1 * scale;

            Vector3f camDir = camera.dir();
            Vector3f camRig = camera.right();
            camDir.y() = 0;
            camRig.y() = 0;
            camDir.normalize();
            camRig.normalize();
            Vector3f A = camera.position();
            A.y() = 0;
            Vector3f B = A + camDir * triangleHeight + camRig * triangleWidth / 2.0;
            Vector3f C = A + camDir * triangleHeight + camRig * triangleWidth / -2.0;
            Vector3f D = A + camDir * smallerTriangleHeight + camRig * smallerTriangleWidth / 2.0;
            Vector3f E = A + camDir * smallerTriangleHeight + camRig * smallerTriangleWidth / -2.0;
            Vector2f minPos = Vector2f(min(min(A.x(), B.x()), C.x()), min(min(A.z(), B.z()), C.z()));
            Vector2f maxPos = Vector2f(max(max(A.x(), B.x()), C.x()), max(max(A.z(), B.z()), C.z()));
            Vector2f mSize = map.getMapSize();

            

            for (int z = minPos.y(); z < maxPos.y(); z += spacing) {
                for (int x = minPos.x(); x < maxPos.x(); x += spacing) {
                    if (PointInTriangle(Vector2f(x, z), Vector2f(A.x(), A.z()), Vector2f(B.x(), B.z()), Vector2f(C.x(), C.z())) &&
                        !PointInTriangle(Vector2f(x, z), Vector2f(A.x(), A.z()), Vector2f(D.x(), D.z()), Vector2f(E.x(), E.z()))) {
                        float xCord = ((int)(x / spacing)) * spacing;
                        float zCord = ((int)(z / spacing)) * spacing;

                        float sizeScale = sqrt((A - Vector3f(xCord, 0, zCord)).norm()) / 2;
                        Matrix4f S = GraphicsUtility::scaleMatrix(Vector3f(sizeScale, sizeScale / 2, sizeScale));
                        if (xCord < mSize.x() / 2 && xCord > mSize.x() / -2 && zCord < mSize.y() / 2 && zCord > mSize.y() / -2) {
                            if (map.getHeightAt(xCord, zCord) > 210 && map.getHeightAt(xCord, zCord) < 255) {
                                iGrassActor.addInstance(GraphicsUtility::translationMatrix(Vector3f(xCord, map.getHeightAt(xCord, zCord), zCord)) * S);
                            }
                        }
                    }
                }
            }
        }
    }

    void placeDekoElements(TerrainMap &map, InstanceActor &iPineActor, InstanceActor &iPineLeavesActor, InstanceActor &iTreeActor, InstanceActor &iTreeLeavesActor, InstanceActor &iPalmActor, InstanceActor &iPalmLeavesActor, InstanceActor &iRockActor) {
        iPineActor.clearInstances();
        iPineLeavesActor.clearInstances();
        iTreeActor.clearInstances();
        iTreeLeavesActor.clearInstances();
        iPalmActor.clearInstances();
        iPalmLeavesActor.clearInstances();
        iRockActor.clearInstances();

        siv::PerlinNoise pNoise;
        float noiseScale = 0.01;

        int ammount = 45;
        Matrix4f S = GraphicsUtility::scaleMatrix(Vector3f(4, 4, 4));
        Matrix4f R = GraphicsUtility::rotationMatrix(static_cast<Quaternionf>(AngleAxisf(GraphicsUtility::degToRad(randomF(0, 360)), Vector3f::UnitY())));
        for (int x = 0; x < ammount; x++) {
            for (int z = 0; z < ammount; z++) {
                float xCord = (x - ammount / 2.0 + 0.5) * map.getMapSize().x() / (float)ammount * 0.9;
                float zCord = (z - ammount / 2.0 + 0.5) * map.getMapSize().y() / (float)ammount * 0.9;

                xCord += randomF(map.getMapSize().x() / (float)ammount / -2.0, map.getMapSize().x() / (float)ammount / 2.0);
                zCord += randomF(map.getMapSize().y() / (float)ammount / -2.0, map.getMapSize().y() / (float)ammount / 2.0);

                R = GraphicsUtility::rotationMatrix(static_cast<Quaternionf>(AngleAxisf(GraphicsUtility::degToRad(randomF(0, 360)), Vector3f::UnitY())));
                if (pNoise.accumulatedOctaveNoise3D(xCord * noiseScale, 0, zCord * noiseScale, 1) < 0.0) {
                    if (map.getHeightAt(xCord, zCord) > 225 && map.getHeightAt(xCord, zCord) < 255) {
                        iPineActor.addInstance(GraphicsUtility::translationMatrix(Vector3f(xCord, map.getHeightAt(xCord, zCord), zCord)) * S);
                        iPineLeavesActor.addInstance(GraphicsUtility::translationMatrix(Vector3f(xCord, map.getHeightAt(xCord, zCord), zCord)) * S);
                    }

                    else if (map.getHeightAt(xCord, zCord) > 202 && map.getHeightAt(xCord, zCord) < 208) {
                        iPalmActor.addInstance(GraphicsUtility::translationMatrix(Vector3f(xCord, map.getHeightAt(xCord, zCord) - 0.3, zCord)) * R * S);
                        iPalmLeavesActor.addInstance(GraphicsUtility::translationMatrix(Vector3f(xCord, map.getHeightAt(xCord, zCord) - 0.3, zCord)) * R * S);
                    }
                }
                if (pNoise.accumulatedOctaveNoise3D(xCord * noiseScale, 0, zCord * noiseScale, 1) > 0.0) {
                    if (map.getHeightAt(xCord, zCord) > 208 && map.getHeightAt(xCord, zCord) < 235) {
                        iTreeActor.addInstance(GraphicsUtility::translationMatrix(Vector3f(xCord, map.getHeightAt(xCord, zCord), zCord)) * R * S);
                        iTreeLeavesActor.addInstance(GraphicsUtility::translationMatrix(Vector3f(xCord, map.getHeightAt(xCord, zCord), zCord)) * R * S);
                    }
                }
            }
        }

        S = GraphicsUtility::scaleMatrix(Vector3f(4, 4, 4));
        ammount = 3;
        for (int x = 0; x < ammount; x++) {
            for (int z = 0; z < ammount; z++) {
                float xCord = (x - ammount / 2.0 + 0.5) * map.getMapSize().x() / (float)ammount * 0.9;
                float zCord = (z - ammount / 2.0 + 0.5) * map.getMapSize().y() / (float)ammount * 0.9;

                xCord += randomF(map.getMapSize().x() / (float)ammount / -2.0, map.getMapSize().x() / (float)ammount / 2.0);
                zCord += randomF(map.getMapSize().y() / (float)ammount / -2.0, map.getMapSize().y() / (float)ammount / 2.0);
                if (map.getHeightAt(xCord, zCord) > 210 && map.getHeightAt(xCord, zCord) < 275) {
                    R = GraphicsUtility::rotationMatrix(static_cast<Quaternionf>(AngleAxisf(GraphicsUtility::degToRad(randomF(0, 360)), Vector3f::UnitY()) * AngleAxisf(GraphicsUtility::degToRad(randomF(0, 360)), Vector3f::UnitX())));
                    iRockActor.addInstance(GraphicsUtility::translationMatrix(Vector3f(xCord, map.getHeightAt(xCord, zCord), zCord)) * R * S);
                }
            }
        }
    }

    void loadNewDekoObjects(bool generateNew, DekoMesh &PineMesh, DekoMesh &PineLeavesMesh, DekoMesh &PalmMesh, DekoMesh &PalmLeavesMesh, DekoMesh &TreeMesh, DekoMesh &TreeLeavesMesh, DekoMesh &GrassMesh, DekoMesh &RockMesh) {
        if (generateNew) {
            TreeGenerator::generateTrees(TreeGenerator::Needle, 1, "Assets/needleTree");
            TreeGenerator::generateTrees(TreeGenerator::Palm, 1, "Assets/palmTree");
            TreeGenerator::generateTrees(TreeGenerator::Normal, 1, "Assets/tree");
            GrassGenerator::generateGrass(GrassType::triangle, 1, "Assets/grass");
            RockGenerator::generateRocks(RockGenerator::Normal, 1, "Assets/rock");
        }
        
        PineMesh.clear();
        PineMesh.load("Assets/needleTree0.obj");
        PineMesh.getMaterial(0)->TexAlbedo = "Assets/richard/Dark_Bark_baseColor.jpg";
        PineMesh.getMaterial(0)->TexNormal = "Assets/richard/Bark_06_normal.jpg";
        PineMesh.getMaterial(0)->TexDepth = "Assets/richard/Bark_06_Packed.png";
        PineMesh.getMaterial(0)->VertexShaderSources.push_back("Shader/InstancePineShader.vert");
        PineMesh.getMaterial(0)->FragmentShaderSources.push_back("Shader/InstanceShader.frag");

        PineLeavesMesh.clear();
        PineLeavesMesh.load("Assets/needleTreeLeaves0.obj");
        PineLeavesMesh.getMaterial(0)->TexAlbedo = "Assets/richard/needle_leaves_color_bright.png";
        PineLeavesMesh.getMaterial(0)->TexDepth = "Assets/richard/needle_leaves_alpha.png";
        PineLeavesMesh.getMaterial(0)->VertexShaderSources.push_back("Shader/InstancePineShader.vert");
        PineLeavesMesh.getMaterial(0)->FragmentShaderSources.push_back("Shader/InstanceGrassShader.frag");

        PalmMesh.clear();
        PalmMesh.load("Assets/palmTree0.obj");
        PalmMesh.getMaterial(0)->TexAlbedo = "Assets/richard/palm_color.jpg";
        PalmMesh.getMaterial(0)->TexNormal = "Assets/richard/Bark_06_normal.jpg";
        PalmMesh.getMaterial(0)->TexDepth = "Assets/richard/Bark_06_Packed.png";
        PalmMesh.getMaterial(0)->VertexShaderSources.push_back("Shader/InstancePineShader.vert");
        PalmMesh.getMaterial(0)->FragmentShaderSources.push_back("Shader/InstanceShader.frag");

        PalmLeavesMesh.clear();
        PalmLeavesMesh.load("Assets/palmTreeLeaves0.obj");
        PalmLeavesMesh.getMaterial(0)->TexAlbedo = "Assets/richard/pla2.png";
        PalmLeavesMesh.getMaterial(0)->TexDepth = "Assets/richard/pla3.png";
        PalmLeavesMesh.getMaterial(0)->VertexShaderSources.push_back("Shader/InstancePineShader.vert");
        PalmLeavesMesh.getMaterial(0)->FragmentShaderSources.push_back("Shader/InstanceGrassShader.frag");

        TreeMesh.clear();
        TreeMesh.load("Assets/tree0.obj");
        TreeMesh.getMaterial(0)->TexAlbedo = "Assets/richard/Bark_06_baseColor.jpg";
        TreeMesh.getMaterial(0)->TexNormal = "Assets/richard/Bark_06_normal.jpg";
        TreeMesh.getMaterial(0)->TexDepth = "Assets/richard/Bark_06_Packed.png";
        TreeMesh.getMaterial(0)->VertexShaderSources.push_back("Shader/InstancePineShader.vert");
        TreeMesh.getMaterial(0)->FragmentShaderSources.push_back("Shader/InstanceShader.frag");

        TreeLeavesMesh.clear();
        TreeLeavesMesh.load("Assets/treeLeaves0.obj");
        TreeLeavesMesh.getMaterial(0)->TexAlbedo = "Assets/richard/leaves3_color.png";
        TreeLeavesMesh.getMaterial(0)->TexDepth = "Assets/richard/leaves3_alpha.jpg";
        TreeLeavesMesh.getMaterial(0)->VertexShaderSources.push_back("Shader/InstancePineShader.vert");
        TreeLeavesMesh.getMaterial(0)->FragmentShaderSources.push_back("Shader/InstanceGrassShader.frag");

        GrassMesh.clear();
        GrassMesh.load("Assets/grass0.obj");
        GrassMesh.getMaterial(0)->TexAlbedo = "Assets/richard/grass_color.jpg";
        GrassMesh.getMaterial(0)->TexDepth = "Assets/richard/grassAlpha.png";
        GrassMesh.getMaterial(0)->VertexShaderSources.push_back("Shader/InstanceGrassShader.vert");
        GrassMesh.getMaterial(0)->FragmentShaderSources.push_back("Shader/InstanceGrassShader.frag");

        RockMesh.clear();
        RockMesh.load("Assets/rock0.obj");
        RockMesh.getMaterial(0)->TexAlbedo = "Assets/richard/Rock_035_baseColor.jpg";
        RockMesh.getMaterial(0)->TexNormal = "Assets/richard/Rock_035_normal.jpg";
        RockMesh.getMaterial(0)->TexDepth = "Assets/richard/Rock_035_Packed.png";
        RockMesh.getMaterial(0)->VertexShaderSources.push_back("Shader/InstanceShader.vert");
        RockMesh.getMaterial(0)->FragmentShaderSources.push_back("Shader/InstanceRockShader.frag");
    }



    void TerrainSetup() {
        bool wireframe = false;
        bool debugTexture = false;
        bool shadows = true;
        bool richard = false;
        bool erode = false;
        bool cameraMode = true;

        if (richard) {
            DecoSetup();
            return;
        }

        GLWindow window;
        RenderDevice renderDevice;
        VirtualCamera camera;
        DirectionalLight sun, light;
        initCForge(&window, &renderDevice, &camera, &sun, &light);

        SGNTransformation rootTransform;
        rootTransform.init(nullptr);

        ClipMap::ClipMapConfig clipMapConfig = {.sideLength = 128, .levelCount = 5};
        HeightMap::NoiseConfig noiseConfig = {.seed = static_cast<uint32_t>(rand()),
            .scale = 1.0f,
            .octaves = 10,
            .persistence = 0.5f,
            .lacunarity = 2.0f};
        HeightMap::HeightMapConfig heightMapConfig = {.width = 1024 / 1, .height = 1024 / 1,
                                                      .mapHeight = 400, .noiseConfig = noiseConfig};

        TerrainMap map = TerrainMap(&rootTransform);
        map.generateClipMap(clipMapConfig);
        map.generateHeightMap(heightMapConfig);

        SceneGraph sceneGraph;

        bool generateNew = true;
        bool renderGrass = true;

        DekoMesh PineMesh;
        InstanceActor iPineActor;

        DekoMesh PineLeavesMesh;
        InstanceActor iPineLeavesActor;

        DekoMesh PalmMesh;
        InstanceActor iPalmActor;

        DekoMesh PalmLeavesMesh;
        InstanceActor iPalmLeavesActor;

        DekoMesh TreeMesh;
        InstanceActor iTreeActor;

        DekoMesh TreeLeavesMesh;
        InstanceActor iTreeLeavesActor;

        DekoMesh GrassMesh;
        InstanceActor iGrassActor;
        
        DekoMesh RockMesh;
        InstanceActor iRockActor;

        loadNewDekoObjects(generateNew, PineMesh, PineLeavesMesh, PalmMesh, PalmLeavesMesh, TreeMesh, TreeLeavesMesh, GrassMesh, RockMesh);

        placeDekoElements(map, iPineActor, iPineLeavesActor, iTreeActor, iTreeLeavesActor, iPalmActor, iPalmLeavesActor, iRockActor);
        iPineActor.init(&PineMesh);
        iPineLeavesActor.init(&PineLeavesMesh);
        iTreeActor.init(&TreeMesh);
        iTreeLeavesActor.init(&TreeLeavesMesh);
        iPalmActor.init(&PalmMesh);
        iPalmLeavesActor.init(&PalmLeavesMesh);
        iRockActor.init(&RockMesh);
        iGrassActor.init(&GrassMesh);
        
        //wind
		Vector3f windVec = Vector3f(1, 0, 0);
		float windAngle = 0;
		float windStr = 1.5;
		float windAngleVariation = 0;
		float windAngleAcc = 100;
		unsigned int windUBO;
		glGenBuffers(1, &windUBO);
		glBindBuffer(GL_UNIFORM_BUFFER, windUBO);
		glBufferData(GL_UNIFORM_BUFFER, sizeof(float) * 4, NULL, GL_STATIC_DRAW);
		glBindBuffer(GL_UNIFORM_BUFFER, 0);
		glBindBufferBase(GL_UNIFORM_BUFFER, 4, windUBO);
		setWindUBO(windUBO, windVec, 0);
        srand((unsigned int)time(NULL));

        sceneGraph.init(&rootTransform);

        ScreenQuad quad;
        initDebugQuad(&quad);

        //fps counter
        int32_t FPSCount = 0;
        clock_t current_ticks, delta_ticks;
        clock_t fps = 60;

        while (!window.shutdown()) {
            current_ticks = clock(); //for fps counter

            // wind
            windAngleVariation += randomF(-windAngleAcc, windAngleAcc) / (float)fps;
            windAngleVariation *= 0.8;
            windAngle += windAngleVariation / (float)fps;
            windVec.x() = cos(windAngle) * windStr;
            windVec.z() = sin(windAngle) * windStr;
            setWindUBO(windUBO, windVec, current_ticks / 60.0);

            window.update();

            sceneGraph.update(1.0f);

            if (erode) {
                map.erode();
            }

            map.update(camera.position().x(), camera.position().z());

            if (shadows) {
                renderDevice.activePass(RenderDevice::RENDERPASS_SHADOW, &sun);
                sceneGraph.render(&renderDevice);
            }

            renderDevice.activePass(RenderDevice::RENDERPASS_GEOMETRY);

            if (wireframe) {
                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                glLineWidth(1);
                sceneGraph.render(&renderDevice);
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            } else {
                sceneGraph.render(&renderDevice);
            }

            //Deko instances
            iPineActor.render(&renderDevice);
            iTreeActor.render(&renderDevice);
            iPalmActor.render(&renderDevice);
            iRockActor.render(&renderDevice);

            if (renderGrass) {
                updateGrass(iGrassActor, map, camera);
                iGrassActor.init(&GrassMesh);
            }

            glDisable(GL_CULL_FACE);
            if (renderGrass) {
                iGrassActor.render(&renderDevice);
            }
            iPineLeavesActor.render(&renderDevice);
            iTreeLeavesActor.render(&renderDevice);
            iPalmLeavesActor.render(&renderDevice);
            glEnable(GL_CULL_FACE);

            renderDevice.activePass(RenderDevice::RENDERPASS_LIGHTING);

            renderDevice.activePass(RenderDevice::RENDERPASS_FORWARD);

            if (debugTexture) {
                glActiveTexture(GL_TEXTURE0);
                map.bindTexture();
                quad.render(&renderDevice);
            }

            window.swapBuffers();

            updateCamera(window.mouse(), window.keyboard(), renderDevice.activeCamera());

            if (cameraMode) {
                camera.position(Vector3f(camera.position().x(),
                                         map.getHeightAt(camera.position().x(), camera.position().z()) + 10,
                                         camera.position().z()));
            }

            if (window.keyboard()->keyPressed(Keyboard::KEY_ESCAPE)) {
                window.closeWindow();
            }
            if (window.keyboard()->keyPressed(Keyboard::KEY_F1)) {
                window.keyboard()->keyState(Keyboard::KEY_F1, Keyboard::KEY_RELEASED);
                wireframe = !wireframe;
            }
            if (window.keyboard()->keyPressed(Keyboard::KEY_F2)) {
                window.keyboard()->keyState(Keyboard::KEY_F2, Keyboard::KEY_RELEASED);
                debugTexture = !debugTexture;
            }
            if (window.keyboard()->keyPressed(Keyboard::KEY_F3)) {
                window.keyboard()->keyState(Keyboard::KEY_F3, Keyboard::KEY_RELEASED);
                clipMapConfig = {.sideLength = 256, .levelCount = 5};

                map.generateClipMap(clipMapConfig);
            }
            if (window.keyboard()->keyPressed(Keyboard::KEY_F4)) {
                window.keyboard()->keyState(Keyboard::KEY_F4, Keyboard::KEY_RELEASED);
                noiseConfig = {.seed = static_cast<uint32_t>(rand()),
                    .scale = 1.0f,
                    .octaves = 10,
                    .persistence = 0.5f,
                    .lacunarity = 2.0f};
                heightMapConfig = {.width = 1024 / 1, .height = 1024 / 1,
                    .mapHeight = 400, .noiseConfig = noiseConfig};
                map.generateHeightMap(heightMapConfig);
                placeDekoElements(map, iPineActor, iPineLeavesActor, iTreeActor, iTreeLeavesActor, iPalmActor, iPalmLeavesActor, iRockActor);
                iPineActor.init(&PineMesh);
                iPineLeavesActor.init(&PineLeavesMesh);
                iTreeActor.init(&TreeMesh);
                iTreeLeavesActor.init(&TreeLeavesMesh);
                iPalmActor.init(&PalmMesh);
                iPalmLeavesActor.init(&PalmLeavesMesh);
                iRockActor.init(&RockMesh);
            }
            if (window.keyboard()->keyPressed(Keyboard::KEY_F5)) {
                window.keyboard()->keyState(Keyboard::KEY_F5, Keyboard::KEY_RELEASED);
                placeDekoElements(map, iPineActor, iPineLeavesActor, iTreeActor, iTreeLeavesActor, iPalmActor, iPalmLeavesActor, iRockActor);
                iPineActor.init(&PineMesh);
                iPineLeavesActor.init(&PineLeavesMesh);
                iTreeActor.init(&TreeMesh);
                iTreeLeavesActor.init(&TreeLeavesMesh);
                iPalmActor.init(&PalmMesh);
                iPalmLeavesActor.init(&PalmLeavesMesh);
                iRockActor.init(&RockMesh);
            }
            if (window.keyboard()->keyPressed(Keyboard::KEY_F6)) {
                window.keyboard()->keyState(Keyboard::KEY_F6, Keyboard::KEY_RELEASED);
                cameraMode = !cameraMode;
            }
            if (window.keyboard()->keyPressed(Keyboard::KEY_F7)) {
                window.keyboard()->keyState(Keyboard::KEY_F7, Keyboard::KEY_RELEASED);
                erode = !erode;
            }
            if (window.keyboard()->keyPressed(Keyboard::KEY_F8)) {
                window.keyboard()->keyState(Keyboard::KEY_F8, Keyboard::KEY_RELEASED);
                iPineActor.clearInstances();
                iPineLeavesActor.clearInstances();
                iTreeActor.clearInstances();
                iTreeLeavesActor.clearInstances();
                iPalmActor.clearInstances();
                iPalmLeavesActor.clearInstances();
                iRockActor.clearInstances();
                iPineActor.init(&PineMesh);
                iPineLeavesActor.init(&PineLeavesMesh);
                iTreeActor.init(&TreeMesh);
                iTreeLeavesActor.init(&TreeLeavesMesh);
                iPalmActor.init(&PalmMesh);
                iPalmLeavesActor.init(&PalmLeavesMesh);
                iRockActor.init(&RockMesh);
            }
            if (window.keyboard()->keyPressed(Keyboard::KEY_F9)) {
                window.keyboard()->keyState(Keyboard::KEY_F9, Keyboard::KEY_RELEASED);
                renderGrass = !renderGrass;
            }
            if (false && window.keyboard()->keyPressed(Keyboard::KEY_F10)) {
                window.keyboard()->keyState(Keyboard::KEY_F10, Keyboard::KEY_RELEASED);
                loadNewDekoObjects(generateNew, PineMesh, PineLeavesMesh, PalmMesh, PalmLeavesMesh, TreeMesh, TreeLeavesMesh, GrassMesh, RockMesh);

                placeDekoElements(map, iPineActor, iPineLeavesActor, iTreeActor, iTreeLeavesActor, iPalmActor, iPalmLeavesActor, iRockActor);
                iPineActor.init(&PineMesh);
                iPineLeavesActor.init(&PineLeavesMesh);
                iTreeActor.init(&TreeMesh);
                iTreeLeavesActor.init(&TreeLeavesMesh);
                iPalmActor.init(&PalmMesh);
                iPalmLeavesActor.init(&PalmLeavesMesh);
                iRockActor.init(&RockMesh);
            }

            //FPS counter
            FPSCount++;
            delta_ticks = clock() - current_ticks;
            if (delta_ticks > 0)
                fps = CLOCKS_PER_SEC / delta_ticks;
            if (FPSCount % 60 == 0) {
                cout << fps << endl;
            }
        }
    }
}

