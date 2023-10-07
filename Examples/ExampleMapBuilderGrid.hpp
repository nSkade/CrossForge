
#ifndef __CFORGE_EXAMPLEMAPBUILDERGRID_HPP__
#define __CFORGE_EXAMPLEMAPBUILDERGRID_HPP__

#include "ExampleSceneBase.hpp"
#include "../CForge/Graphics/Actors/SkyboxActor.h"
#include "../CForge/MeshProcessing/PrimitiveShapeFactory.h"

#include "fcl/narrowphase/collision_object.h"
#include "fcl/narrowphase/distance.h"


using namespace Eigen;
using namespace std;

namespace CForge {

	class ExampleMapBuilderGrid : public ExampleSceneBase {
	public:
		ExampleMapBuilderGrid(void) {
			m_WindowTitle = "CrossForge Example MapBuilder Grid Variant";
		}//Constructor

		~ExampleMapBuilderGrid(void) {
			clear();
		}//Destructor

		void init(void) override {
			initWindowAndRenderDevice();
			initCameraAndLights();

			srand(static_cast <unsigned> (time(0)));

			// build scene graph	
			m_RootSGN.init(nullptr);
			m_SG.init(&m_RootSGN); 

			T3DMesh<float> M;
			// load buildings
			//weiß hoch
			SAssetIO::load("MyAssets/Buildings/building_06/scene.gltf", &M);
			setMeshShader(&M, 0.1f, 0.04f);
			M.computePerVertexNormals();
			m_Buildings[0].init(&M);
			M.clear();
			//weiß hoch mit kante
			SAssetIO::load("MyAssets/Buildings/building_07/scene.gltf", &M);
			setMeshShader(&M, 0.1f, 0.04f);
			M.computePerVertexNormals();
			m_Buildings[1].init(&M);
			M.clear();
			//rot klein
			SAssetIO::load("MyAssets/Buildings/building_08/scene.gltf", &M);
			setMeshShader(&M, 0.1f, 0.04f);
			M.computePerVertexNormals();
			m_Buildings[2].init(&M);
			M.clear();

			SAssetIO::load("MyAssets/cube.obj", &M);
			setMeshShader(&M, 0.1f, 0.04f);
			M.computePerVertexNormals();
			m_Cube.init(&M);
			M.clear();

			m_BuildingGroupSGN.init(&m_RootSGN);
			m_GridTileGroupSGN.init(&m_RootSGN);

			//Textures for second Skybox City 1
			/// gather textures for the skyboxes
			m_City1.push_back("MyAssets/FlappyAssets/skybox1/right.bmp");
			m_City1.push_back("MyAssets/FlappyAssets/skybox1/left.bmp");
			m_City1.push_back("MyAssets/FlappyAssets/skybox1/up.bmp");
			m_City1.push_back("MyAssets/FlappyAssets/skybox1/down.bmp");
			m_City1.push_back("MyAssets/FlappyAssets/skybox1/back.bmp");
			m_City1.push_back("MyAssets/FlappyAssets/skybox1/front.bmp");
			//Textures for Skybox City 2
			m_City2.push_back("MyAssets/FlappyAssets/skybox/right.bmp");
			m_City2.push_back("MyAssets/FlappyAssets/skybox/left.bmp");
			m_City2.push_back("MyAssets/FlappyAssets/skybox/up.bmp");
			m_City2.push_back("MyAssets/FlappyAssets/skybox/down.bmp");
			m_City2.push_back("MyAssets/FlappyAssets/skybox/back.bmp");
			m_City2.push_back("MyAssets/FlappyAssets/skybox/front.bmp");
			
			// create actor and initialize
			m_Skybox.init(m_City1[0], m_City1[1], m_City1[2], m_City1[3], m_City1[4], m_City1[5]);

			// set initialize color adjustment values
			m_Skybox.brightness(1.15f);
			m_Skybox.contrast(1.1f);
			m_Skybox.saturation(1.2f);

			// create scene graph for the Skybox
			m_SkyboxTransSGN.init(nullptr);
			m_SkyboxGeomSGN.init(&m_SkyboxTransSGN, &m_Skybox);
			m_SkyboxSG.init(&m_SkyboxTransSGN);


			//placeBuilding(0, 60.0f, 1.0f, 1.0f);
			//placeBuilding(1, 120.0f, 1.0f, 1.0f);
			//placeBuilding(2, 180.0f, 1.0f, 1.0f);


			std::string GLError = "";
			CForgeUtility::checkGLError(&GLError);
			if (!GLError.empty()) printf("GLError occurred: %s\n", GLError.c_str());



		}//initialize

		//PlaceBuilding

		void placeBuilding(int variant, float x, float y, float z) {
			SGNTransformation* pTransformSGN = nullptr;
			SGNGeometry* pGeomSGN = nullptr;

			pTransformSGN = new SGNTransformation();
			pTransformSGN->init(&m_BuildingGroupSGN);

			// set to other vector
			pTransformSGN->translation(Vector3f(x, z, y));
			pTransformSGN->scale(m_building_scale);
			if (variant == 2) pTransformSGN->scale(Vector3f(2.5f, 5.0f, 2.5f));

			pGeomSGN = new SGNGeometry();
			pGeomSGN->init(pTransformSGN, &m_Buildings[variant]);

			m_BuildingTransformSGNs.push_back(pTransformSGN);
			m_BuildingGeoSGNs.push_back(pGeomSGN);
		}
		 
		void createGrid() {
			// Setze die Anfangsposition für das aktive Tile
			activeTileRow = 0;
			activeTileCol = 0;

			// Größe und Anzahl der Reihen und Spalten des Grids
			
			const float tileSize = 10.0f; // Die Größe eines Tiles

			// Schleife zur Erstellung und Platzierung der Grid-Tiles
			for (int row = 0; row < numRows; ++row) {
				for (int col = 0; col < numCols; ++col) {
					// Berechne die Position des Tiles
					float x = col * tileSize;
					float z = row * tileSize;

					// Erstelle eine Kopie des Cube-Modells
					SGNTransformation* pTransformSGN = new SGNTransformation();
					pTransformSGN->init(&m_GridTileGroupSGN);
					pTransformSGN->translation(Vector3f(x, 0.0f, z));
					pTransformSGN->scale(Vector3f(1.0f, 0.5f, 1.0f)); // Standard-Skalierung

					SGNGeometry* pGeomSGN = new SGNGeometry();
					pGeomSGN->init(pTransformSGN, &m_Cube);

					// Füge das Tile zur Szene hinzu
					m_GridTilesTransformSGNs.push_back(pTransformSGN);
					m_GridTileGeoSGNs.push_back(pGeomSGN);

					// Skaliere das aktive Tile (wenn es das ist)
					if (row == activeTileRow && col == activeTileCol) {
						pTransformSGN->scale(Vector3f(1.0f, 1.0f, 1.0f));
					}
				}
			}
		}

		void updateActiveTileScaling() {

			for (int i = 0; i < m_GridTilesTransformSGNs.size(); ++i) {
				m_GridTilesTransformSGNs[i]->scale(Vector3f(1.0f, 0.5f, 1.0f)); // Standard-Skalierung
			}
			// Finde das aktive Tile anhand von activeTileRow und activeTileCol
			int activeTileIndex = activeTileRow * numCols + activeTileCol;

			// Setze die Skalierung des aktiven Tiles auf die gewünschten Werte
			if (activeTileIndex >= 0 && activeTileIndex < m_GridTilesTransformSGNs.size()) {
				// Finde das Transformation-SGN des aktiven Tiles
				SGNTransformation* pActiveTileTransform = m_GridTilesTransformSGNs[activeTileIndex];

				// Setze die Skalierung des aktiven Tiles
				pActiveTileTransform->scale(Vector3f(1.0f, 1.0f, 1.0f)); // Hier die gewünschte Skalierung eintragen
			}
		}
		
		void placeBuildingAtActiveTile(int variant) {
			// Stelle sicher, dass die aktive Tile-Position gültig ist
			if (activeTileRow >= 0 && activeTileRow < numRows && activeTileCol >= 0 && activeTileCol < numCols) {
				// Berechne die Position für das Gebäude basierend auf der aktiven Tile-Position
				float x = activeTileCol * 10; // Multipliziere mit der Gebäude-Skalierung in x-Richtung
				float y = activeTileRow * 10; // Multipliziere mit der Gebäude-Skalierung in z-Richtung

				// Rufe die Methode zum Platzieren eines Gebäudes auf
				placeBuilding(variant, x, y, 0.0f);

				// Inkrementiere den Index der ausgewählten Gebäudevariante für das nächste Gebäude
				variant = (variant + 1) % 3; // Annahme: Es gibt 3 Gebäudevaiants

			}
		}

		void handleBuildingPlacementInput() {
			// Überprüfe, ob eine Taste für den Gebäudeplatzierung eingegeben wurde
			if (m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_B, true)) {
				// Platzieren Sie das ausgewählte Gebäude am aktiven Tile
				placeBuildingAtActiveTile(selectedBuildingVariant);

				// Erhöhen Sie den Index des ausgewählten Gebäudevaiants für das nächste Gebäude
				selectedBuildingVariant = (selectedBuildingVariant + 1) % 3; // Annahme: Es gibt 3 Gebäudevaiants
			}
		}

		


		enum CameraMode {
			Bird,
			Default
		};

		CameraMode currentCameraMode = CameraMode::Default;

		void updateCamera(Keyboard* keyBoard) {
			Keyboard* pKeyboard = keyBoard;


			// Check for the key to toggle camera mode
			if (pKeyboard->keyPressed(Keyboard::KEY_L, true)) {
				// Toggle camera mode here
				if (currentCameraMode == CameraMode::Default) {
					currentCameraMode = CameraMode::Bird;
				}
				else if (currentCameraMode == CameraMode::Bird) {
					currentCameraMode = CameraMode::Default;
				}
			}

			if (currentCameraMode == CameraMode::Default) {
				defaultCameraUpdate(&m_Cam, m_RenderWin.keyboard(), m_RenderWin.mouse());
			}
		}


		void mainLoop(void)override {
			if (!glLoaded) {
				gladLoadGL();
				glLoaded = true;
			}

			m_RenderWin.update();
			m_SG.update(60.0f / m_FPS);
			m_SkyboxSG.update(60.0f / m_FPS);

			// handle input for the skybox
			Keyboard* pKeyboard = m_RenderWin.keyboard();
			float Step = (pKeyboard->keyPressed(Keyboard::KEY_LEFT_SHIFT)) ? -0.05f : 0.05f;

			if (pKeyboard->keyPressed(Keyboard::KEY_1, true)) m_Skybox.brightness(m_Skybox.brightness() + Step);
			if (pKeyboard->keyPressed(Keyboard::KEY_2, true)) m_Skybox.saturation(m_Skybox.saturation() + Step);
			if (pKeyboard->keyPressed(Keyboard::KEY_3, true)) m_Skybox.contrast(m_Skybox.contrast() + Step);
			if (pKeyboard->keyPressed(Keyboard::KEY_LEFT_SHIFT) && pKeyboard->keyPressed(Keyboard::KEY_R)) {
				m_SkyboxTransSGN.rotationDelta(Quaternionf::Identity());
			}
			else if (pKeyboard->keyPressed(Keyboard::KEY_R, true)) {
				Quaternionf RDelta;
				RDelta = AngleAxisf(CForgeMath::degToRad(-2.5f / 60.0f), Vector3f::UnitY());
				m_SkyboxTransSGN.rotationDelta(RDelta);
			}
			
			//CAMERA SWITCH 
			updateCamera(pKeyboard);
			//SKYBOX SWITCH
			if (pKeyboard->keyPressed(Keyboard::KEY_F1, true)) m_Skybox.init(m_City1[0], m_City1[1], m_City1[2], m_City1[3], m_City1[4], m_City1[5]);
			if (pKeyboard->keyPressed(Keyboard::KEY_F2, true)) m_Skybox.init(m_City2[0], m_City2[1], m_City2[2], m_City2[3], m_City2[4], m_City2[5]);
			
			if (m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_G, true)) {
				// Erstelle das Grid, wenn die Leertaste gedrückt wird
				createGrid();
			}
			if (m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_LEFT, true)) {
				// Bewege das aktive Tile nach links (col verringern)
				if (activeTileCol > 0) {
					activeTileCol--;
					updateActiveTileScaling();
				}
			}
			else if (m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_RIGHT, true)) {
				// Bewege das aktive Tile nach rechts (col erhöhen)
				if (activeTileCol < 3) { // 4 Spalten insgesamt (0, 1, 2, 3)
					activeTileCol++;
					updateActiveTileScaling();
				}
			}
			else if (m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_UP, true)) {
				// Bewege das aktive Tile nach oben (row verringern)
				if (activeTileRow > 0) {
					activeTileRow--;
					updateActiveTileScaling();
				}
			}
			else if (m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_DOWN, true)) {
				// Bewege das aktive Tile nach unten (row erhöhen)
				if (activeTileRow < 3) { // 4 Reihen insgesamt (0, 1, 2, 3)
					activeTileRow++;
					updateActiveTileScaling();
				}
			}
			handleBuildingPlacementInput();

			m_RenderDev.activePass(RenderDevice::RENDERPASS_SHADOW, &m_Sun);
			m_RenderDev.activeCamera((VirtualCamera*)m_Sun.camera());
			m_SG.render(&m_RenderDev);
			//m_CheckpointsSG.render(&m_RenderDev);

			m_RenderDev.activePass(RenderDevice::RENDERPASS_GEOMETRY);
			m_RenderDev.activeCamera(&m_Cam);
			m_SG.render(&m_RenderDev);

			m_RenderDev.activePass(RenderDevice::RENDERPASS_LIGHTING);

			m_RenderDev.activePass(RenderDevice::RENDERPASS_FORWARD, nullptr, false);

			glEnable(GL_BLEND);
			glBlendFunc(GL_ONE, GL_ONE);

			if (activeTile)
				glColorMask(false, true, false, true);
			else
				glColorMask(true, false, false, true);

			
			glColorMask(true, true, true, true);
			glDisable(GL_BLEND);

			// Skybox should be last thing to render
			m_SkyboxSG.render(&m_RenderDev);

			m_RenderWin.swapBuffers();

			updateFPS();
			defaultKeyboardUpdate(m_RenderWin.keyboard());
		}

	protected:

		SkeletalActor m_Bird;
		SGNTransformation m_RootSGN;



		SkyboxActor m_Skybox;
		SceneGraph m_SkyboxSG;
		SGNTransformation m_SkyboxTransSGN;
		SGNGeometry m_SkyboxGeomSGN;

		vector<string> m_City1;
		vector<string> m_City2;

		StaticActor m_Ground;
		SGNGeometry m_GroundSGN;
		SGNTransformation m_GroundTransformSGN;

		//Building
		StaticActor m_Buildings[3];
		SGNTransformation m_BuildingGroupSGN;
		vector<SGNTransformation*> m_BuildingTransformSGNs; // List to hold building transformation SGNs
		vector<SGNGeometry*> m_BuildingGeoSGNs;   // List to hold building geometry SGNs
		const Vector3f m_building_scale = Vector3f(5.0f, 5.0f, 5.0f);

		SGNTransformation m_GridTileGroupSGN;
		vector<SGNTransformation*> m_GridTilesTransformSGNs;
		vector<SGNGeometry*> m_GridTileGeoSGNs;
		StaticActor m_Cube;
		bool m_paused = true;

		bool activeTile = false;
		bool glLoaded = false;


		int activeTileRow = 0;
		int activeTileCol = 0;
	
		const int numRows = 4;
		const int numCols = 4;
		
		int selectedBuildingVariant = 0; // Index des ausgewählten Gebäudevaiants

		


		

	};//ExampleMapBuilderGrid

}

#endif