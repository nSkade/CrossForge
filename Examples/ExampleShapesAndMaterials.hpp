/*****************************************************************************\
*                                                                           *
* File(s): ExampleShapesAndMaterials.hpp                                            *
*                                                                           *
* Content:           *
*                       *
*                                                                           *
*                                                                           *
* Author(s): Tom Uhlmann                                                    *
*                                                                           *
*                                                                           *
* The file(s) mentioned above are provided as is under the terms of the     *
* MIT License without any warranty or guaranty to work properly.            *
* For additional license, copyright and contact/support issues see the      *
* supplied documentation.                                                   *
*                                                                           *
\****************************************************************************/
#ifndef __CFORGE_EXAMPLESHAPESANDMATERIALS_HPP__
#define __CFORGE_EXAMPLESHAPESANDMATERIALS_HPP__

#include "ExampleSceneBase.hpp"
#include <crossforge/MeshProcessing/PrimitiveShapeFactory.h>

namespace CForge {
	class ExampleShapesAndMaterials : public ExampleSceneBase {
	public:
		ExampleShapesAndMaterials(void) {

		}//Constructor

		~ExampleShapesAndMaterials(void) {

		}//Destructor

		void init(void)override {
			initWindowAndRenderDevice();
			initCameraAndLights();
			initSkybox();
			initFPSLabel();

			m_RootSGN.init(nullptr);
			m_SG.init(&m_RootSGN);


			// create ground plane
			T3DMesh<float> M;
			PrimitiveShapeFactory::plane(&M, Vector2f(150.0f, 150.0f), Vector2i(1, 1));
			setMeshShader(&M, 0.4f, 0.04f);
			M.changeUVTiling(Vector3f(20.0f, 20.0f, 1.0f));
			M.computePerVertexNormals();
			M.computePerVertexTangents();
			CForgeUtility::defaultMaterial(M.getMaterial(0), CForgeUtility::PLASTIC_WHITE);
			M.getMaterial(0)->TexAlbedo = "Assets/ExampleScenes/Textures/Tiles107/Tiles107_1K_f.webp";
			M.getMaterial(0)->TexNormal = "Assets/ExampleScenes/Textures/Tiles107/Tiles107_1K_NormalGL.webp";
			// BoundingVolume BV;
			// m_GroundPlane.init(&M);
			// m_GroundPlane.boundingVolume(BV);
			// m_GroundPlaneSGN.init(&m_RootSGN, &m_GroundPlane);
			// M.clear();

			m_ModelType = 0;
			m_MaterialType = 0;
			m_Wireframe = false;

			// create models
			createObjects(m_ModelType, CForgeUtility::DefaultMaterial(m_MaterialType));

			// and let them rotate about the y-axis (90 degree per second)
			m_ObjectRotation = AngleAxisf(CForgeMath::degToRad(90.0f / 60.0f), Vector3f::UnitY());
			letObjectRotate(m_ObjectRotation);

			m_Sun.position(Vector3f(-50.0f, 100.0f, 200.0f));
			m_Sun.direction(-m_Sun.position().normalized());
			m_Sun.initShadowCasting(4096, 4096, Vector2i(50, 50), 0.5f, 1000.0f);

			m_Cam.position(Vector3f(-9.0f, 7.0f, -9.0f));
			m_Cam.lookAt(m_Cam.position(), Vector3f(0.0f, 3.0f, 0.0f));

			// create help text
			LineOfText* pKeybindings = new LineOfText();
			LineOfText* pKeybindings2 = new LineOfText();
			pKeybindings->init(CForgeUtility::defaultFont(CForgeUtility::FONTTYPE_SANSERIF, 18), "Movement: (Shift) + W,A,S,D  | Rotation: LMB/RMB + Mouse | F1: Toggle help text");
			pKeybindings2->init(CForgeUtility::defaultFont(CForgeUtility::FONTTYPE_SANSERIF, 18), "1: Cycle through shapes | M: Cycle through materials | V: Wireframe shapes");
			pKeybindings->color(0.0f, 0.0f, 0.0f, 1.0f);
			pKeybindings2->color(0.0f, 0.0f, 0.0f, 1.0f);
			m_HelpTexts.push_back(pKeybindings);
			m_HelpTexts.push_back(pKeybindings2);
			m_DrawHelpTexts = true;


		}//initialize

		void clear(void)override {

			ExampleSceneBase::clear();
		}//clear

		void release(void)override {
			delete this;
		}//release

		void mainLoop(void)override {
			m_RenderWin.update();
			m_SG.update(60.0f / m_FPS);
			m_SkyboxSG.update(60.0f / m_FPS);

			defaultCameraUpdate(&m_Cam, m_RenderWin.keyboard(), m_RenderWin.mouse());

			m_RenderDev.activePass(RenderDevice::RENDERPASS_SHADOW, &m_Sun);
			m_RenderDev.activeCamera(const_cast<VirtualCamera*>(m_Sun.camera()));
			m_SG.render(&m_RenderDev);

			m_RenderDev.activePass(RenderDevice::RENDERPASS_FORWARD, nullptr, true);
			m_RenderDev.activeCamera(&m_Cam);
			m_SG.render(&m_RenderDev);
			m_SkyboxSG.render(&m_RenderDev);
			if (m_FPSLabelActive) m_FPSLabel.render(&m_RenderDev);
			if (m_DrawHelpTexts) drawHelpTexts();

			m_RenderWin.swapBuffers();

			updateFPS();

			defaultKeyboardUpdate(m_RenderWin.keyboard());

			if (m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_1, true)) {
				m_ModelType = (m_ModelType + 1) % 7;
				createObjects(m_ModelType, CForgeUtility::DefaultMaterial(m_MaterialType));
				letObjectRotate(m_ObjectRotation);
			}

			if (m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_M, true)) {
				m_MaterialType = (m_MaterialType+ 1) % CForgeUtility::DEFAULT_MATERIAL_COUNT;
				changeMaterial(CForgeUtility::DefaultMaterial(m_MaterialType));
			}

			if (m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_V, true)) {
				m_Wireframe = !m_Wireframe;
				updateVisualization();
			}

			// show one specific object 
			if (m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_P, true)) {
				m_Object_vis += 1 % m_joints.size();
			}

		}//mainLoop

	protected:

		void updateVisualization(void) {
			for (auto i : m_ObjectSGNs) {
				if (nullptr == i) continue;
				i->visualization((m_Wireframe) ? SGNGeometry::VISUALIZATION_WIREFRAME : SGNGeometry::VISUALIZATION_FILL);
			}
		}//updateVisualization

		void letObjectRotate(const Eigen::Quaternionf Speed) {
			for (auto i : m_ObjectTransformSGNs) if (nullptr != i) i->rotationDelta(Speed);
		}//letObjectsRotate

		void changeMaterial(CForgeUtility::DefaultMaterial Mat) {
			for (uint32_t x = 0; x < 10; ++x) {
				for (uint32_t z = 0; z < 10; ++z) {
					uint32_t Index = (x * 10) + z;
					if (m_Objects.size() > Index) {
						CForgeUtility::defaultMaterial(m_Objects[Index]->material(0), Mat);
						m_Objects[Index]->material(0)->roughness(x / 10.0f);
						m_Objects[Index]->material(0)->metallic(z / 10.0f);
					}
				}//for[z]
			}//for[x]

			// last object separately
			if(m_Objects.size() > 0) CForgeUtility::defaultMaterial(m_Objects[m_Objects.size() - 1]->material(0), Mat);

		}//changeMaterial

		void destroyObjects(void) {
			for (auto& i : m_ObjectSGNs) if (nullptr != i) delete i;
			for (auto& i : m_ObjectTransformSGNs) if (nullptr != i) delete i;
			for (auto& i : m_Objects) if (nullptr != i) delete i;

			m_ObjectSGNs.clear();
			m_ObjectTransformSGNs.clear();
			m_Objects.clear();
		}//destroyObjects

		void createObjects(int32_t Object, CForgeUtility::DefaultMaterial Mat) {
			// clean up first
			destroyObjects();

			// create the object to render
			T3DMesh<float> M;
			switch (Object) {
			case 0: PrimitiveShapeFactory::uvSphere(&M, Vector3f(2.0f, 2.0f, 2.0f), 15, 15); break;
			case 1: PrimitiveShapeFactory::plane(&M, Vector2f(2.0f, 2.0f), Vector2i(1.0f, 1.0f)); break;
			case 2: PrimitiveShapeFactory::circle(&M, Vector2f(2.0f, 2.0f), 20, 0.0f); break;
			case 3: PrimitiveShapeFactory::cuboid(&M, Vector3f(2.0f, 2.0f, 2.0f), Vector3i(1, 1, 1)); break;
			case 4: PrimitiveShapeFactory::Torus(&M, 1.0f, 0.5f, 15, 15); break;
			case 5: PrimitiveShapeFactory::cone(&M, Vector3f(2.0f, 2.0f, 2.0f), 20); break;
			case 6: PrimitiveShapeFactory::cylinder(&M, Vector2f(2.0f, 2.0f), Vector2f(2.0f, 2.0f), 2.0f, 20, Vector2f(0.0f, 0.0f)); break;
			default: break;
			}

			setMeshShader(&M, 0.4f, 0.04f);
			M.computePerVertexNormals();
			CForgeUtility::defaultMaterial(M.getMaterial(0), Mat);

		   	SGNTransformation* pTrans2 = new SGNTransformation();
   			pTrans2->init(&m_RootSGN);
   			pTrans2->rotation(Quaternionf(AngleAxis(CForgeMath::degToRad(180.0f), Vector3f::UnitZ())));
			
			for (size_t i = 0; i < m_joints.size(); i++)
            {
                StaticActor* pActor = new StaticActor();
                SGNGeometry* pGeomSGN = new SGNGeometry();
                SGNTransformation* pTransSGN = new SGNTransformation();
             
                pActor->init(&M);
                pTransSGN->init(pTrans2, m_joints[i]);

                pGeomSGN->init(pTransSGN, pActor);
                pGeomSGN->scale(Vector3f(0.008f, 0.008f, 0.008f));

                m_Objects.push_back(pActor);
                m_ObjectSGNs.push_back(pGeomSGN);
                m_ObjectTransformSGNs.push_back(pTransSGN);

            }

			StaticActor* pActorVis = new StaticActor();
            SGNGeometry* pGeomSGNVis = new SGNGeometry();
            SGNTransformation* pTransSGNVis = new SGNTransformation();
             
            pActorVis->init(&M);
			
        	pTransSGNVis->init(pTrans2, m_joints[m_Object_vis]);

            pGeomSGNVis->init(pTransSGNVis, pActorVis);
            pGeomSGNVis->scale(Vector3f(0.0275f, 0.0275f, 0.0275f));

            m_Objects.push_back(pActorVis);
            m_ObjectSGNs.push_back(pGeomSGNVis);
            m_ObjectTransformSGNs.push_back(pTransSGNVis);



			


			

			// create 10 x 10 objects with variation of roughness along the x axis and metallic along the z axis
			/*
			for (uint32_t x = 0; x < 10; ++x) {
				for (uint32_t z = 0; z < 10; ++z) {
					StaticActor* pActor = new StaticActor();
					SGNGeometry* pGeomSGN = new SGNGeometry();
					SGNTransformation* pTransSGN = new SGNTransformation();

					pActor->init(&M);
					pTransSGN->init(&m_RootSGN, Vector3f(x*3.5f, 2.0f, z*3.5f));
					pGeomSGN->init(pTransSGN, pActor);

					m_Objects.push_back(pActor);
					m_ObjectSGNs.push_back(pGeomSGN);
					m_ObjectTransformSGNs.push_back(pTransSGN);

					pActor->material(0)->roughness(x / 10.0f);
					pActor->material(0)->metallic(z / 10.0f);
				}//for[y]
			}//for[x]
			*/

			// at last create one object that shows the original material
			// StaticActor* pActor = new StaticActor();
			// SGNGeometry* pGeomSGN = new SGNGeometry();
			// SGNTransformation* pTransSGN = new SGNTransformation();

			//pActor->init(&M);
			//pTransSGN->init(&m_RootSGN, Vector3f(-4.5f, 4.0f, -4.5f));
			//pTransSGN->scale(Vector3f(2.0f, 2.0f, 2.0f));
			//pGeomSGN->init(pTransSGN, pActor);

			// m_Objects.push_back(pActor);
			// m_ObjectSGNs.push_back(pGeomSGN);
			// m_ObjectTransformSGNs.push_back(pTransSGN);

			updateVisualization();

		}//createObjects

		StaticActor m_GroundPlane;
		SGNGeometry m_GroundPlaneSGN;

		SGNTransformation m_RootSGN;

		std::vector<StaticActor*> m_Objects;
		std::vector<SGNGeometry*> m_ObjectSGNs;
		std::vector<SGNTransformation*> m_ObjectTransformSGNs;

		std::vector<Vector3f> m_joints =  
{{3.94747518e-02,-8.59756827e-01,-1.59036815e-02},
 {1.33411782e-02,-7.14593768e-01,1.16603464e-01},
 {-8.47112536e-02,-7.33641624e-01,7.85898343e-02},
 {-1.58986837e-01,-8.64837289e-01,-1.41850233e-01},
 {-1.64881945e-02,-1.05994523e+00,-1.47088721e-01},
 {1.49049133e-01,-7.27242351e-01,8.83810744e-02},
 {2.98869014e-01,-8.18292737e-01,-1.03780068e-01},
 {2.04316020e-01,-1.04364717e+00,-1.00651540e-01},
 {2.80997343e-03,-3.18577886e-01,5.78146381e-03},
 {-5.89656159e-02,-2.22979665e-01,3.18870274e-03},
 {-7.66289830e-02,1.13951623e-01,-1.19044468e-01},
 {-9.07504037e-02,4.90101844e-01,7.56748021e-03},
 {5.05116582e-02,-2.26575613e-01,1.27169881e-02},
 {7.52922818e-02,1.15379661e-01,-1.38418809e-01},
 {-9.98946279e-03,4.73495126e-01,-3.89736891e-03},
 {1.43547952e-02,-9.05765176e-01,4.07618061e-02},
 {7.56717026e-02,-9.05909300e-01,3.88211682e-02},
 {-2.62719430e-02,-8.75069320e-01,1.04993045e-01},
 {1.09209403e-01,-8.76369894e-01,1.02496915e-01},
 {4.75797616e-02,5.52878857e-01,-1.69851020e-01},
 {9.25203189e-02,5.57806849e-01,-1.08983070e-01},
 {-6.32943958e-03,5.02352655e-01,6.27544373e-02},
 {-6.48626909e-02,5.74756265e-01,-1.49891198e-01},
 {-1.38039052e-01,5.78414381e-01,-1.13141477e-01},
 {-1.01467967e-01,5.29016912e-01,8.01073462e-02},
 {2.04316020e-01,-1.04364717e+00,-1.00651540e-01},
 {1.63600177e-01,-1.06369948e+00,-8.58288333e-02},
 {1.39545605e-01,-1.07812786e+00,-8.20400417e-02},
 {1.26668870e-01,-1.09637928e+00,-7.05917925e-02},
 {1.13608636e-01,-1.12578094e+00,-7.04116970e-02},
 {1.48987755e-01,-1.10415578e+00,-4.65407930e-02},
 {1.35882065e-01,-1.12842226e+00,-3.26305665e-02},
 {1.24388888e-01,-1.14292049e+00,-4.42787074e-02},
 {1.09046370e-01,-1.15677142e+00,-6.05788231e-02},
 {1.63809687e-01,-1.12105870e+00,-4.48983647e-02},
 {1.49139538e-01,-1.14677715e+00,-4.37463410e-02},
 {1.37877643e-01,-1.16269696e+00,-5.61773218e-02},
 {1.22874618e-01,-1.18300879e+00,-7.09559023e-02},
 {1.83420554e-01,-1.12684536e+00,-5.78648746e-02},
 {1.67923003e-01,-1.14899826e+00,-5.64062223e-02},
 {1.56015038e-01,-1.16792893e+00,-6.13732152e-02},
 {1.40398026e-01,-1.18740702e+00,-6.50729388e-02},
 {1.97723478e-01,-1.12894344e+00,-7.19331950e-02},
 {1.90913677e-01,-1.14568985e+00,-7.50557780e-02},
 {1.82497129e-01,-1.15951788e+00,-8.35883692e-02},
 {1.69681847e-01,-1.17670560e+00,-9.22491774e-02},
 {-1.64881945e-02,-1.05994523e+00,-1.47088721e-01},
 {2.14084238e-02,-1.07234263e+00,-1.21073164e-01},
 {4.79707271e-02,-1.07833004e+00,-1.13344908e-01},
 {6.78913742e-02,-1.09037161e+00,-1.03954494e-01},
 {8.72491002e-02,-1.11528325e+00,-1.12289801e-01},
 {2.91549750e-02,-1.11454880e+00,-8.16678926e-02},
 {4.32828441e-02,-1.13421035e+00,-6.24865368e-02},
 {5.67394868e-02,-1.14911723e+00,-5.38470633e-02},
 {7.00704753e-02,-1.16700363e+00,-3.88271809e-02},
 {1.91471986e-02,-1.13401663e+00,-8.70555639e-02},
 {3.41613963e-02,-1.15784895e+00,-7.78604522e-02},
 {4.95409817e-02,-1.17511320e+00,-7.73991868e-02},
 {6.59303069e-02,-1.19876397e+00,-7.22884834e-02},
 {7.74717331e-03,-1.14148223e+00,-1.07068956e-01},
 {2.21678726e-02,-1.16318154e+00,-9.97069478e-02},
 {3.59067991e-02,-1.18151069e+00,-9.93934646e-02},
 {5.17694354e-02,-1.20048606e+00,-9.41788405e-02},
 {6.72278926e-04,-1.14419675e+00,-1.25762582e-01},
 {7.41098356e-03,-1.16121101e+00,-1.27045214e-01},
 {1.86315514e-02,-1.17480636e+00,-1.31954864e-01},
 {3.36588025e-02,-1.19267511e+00,-1.33702114e-01},
 {-1.63669866e-02,-9.06271100e-01,4.31579985e-02},
 {-7.86580984e-03,-9.20788169e-01,2.74494942e-02},
 {5.94473490e-03,-9.23168957e-01,1.42622441e-02},
 {2.05415022e-02,-9.20895576e-01,6.49352372e-03},
 {3.28716487e-02,-9.15837049e-01,4.44684271e-03},
 {5.09513319e-02,-9.15172458e-01,4.36531566e-03},
 {6.37453347e-02,-9.19358969e-01,6.06095605e-03},
 {7.87507892e-02,-9.20669556e-01,1.29776169e-02},
 {9.22982320e-02,-9.17247832e-01,2.61067897e-02},
 {1.00313447e-01,-9.02126312e-01,4.14884761e-02},
 {4.12813909e-02,-8.99569631e-01,2.83406372e-03},
 {4.06839401e-02,-8.88163149e-01,-4.14772239e-03},
 {4.02619913e-02,-8.77678394e-01,-1.08771212e-02},
 {3.97950411e-02,-8.67551506e-01,-1.73027646e-02},
 {2.83521600e-02,-8.56688201e-01,3.05140298e-03},
 {3.35925072e-02,-8.54172945e-01,-4.10708017e-04},
 {3.94378603e-02,-8.51949275e-01,-2.16040621e-03},
 {4.55498546e-02,-8.53895783e-01,-4.23285208e-04},
 {5.10630086e-02,-8.56086731e-01,2.87861144e-03},
 {-2.09865812e-03,-8.98999393e-01,2.70229783e-02},
 {6.34176284e-03,-9.03802335e-01,1.77963786e-02},
 {1.57930031e-02,-9.02952671e-01,1.67087093e-02},
 {2.37628780e-02,-8.96676481e-01,1.92704722e-02},
 {1.59221478e-02,-8.94460440e-01,1.80509631e-02},
 {6.64131669e-03,-8.94457102e-01,1.92372762e-02},
 {5.89270368e-02,-8.95485759e-01,1.87890828e-02},
 {6.71003163e-02,-9.00908053e-01,1.60224997e-02},
 {7.67992958e-02,-9.01283681e-01,1.66639723e-02},
 {8.51570815e-02,-8.96200776e-01,2.56347917e-02},
 {7.57824928e-02,-8.92386913e-01,1.82782859e-02},
 {6.64435029e-02,-8.92922938e-01,1.72284264e-02},
 {1.68294515e-02,-8.34448457e-01,1.47470618e-02},
 {2.37696506e-02,-8.37918758e-01,4.50380705e-03},
 {3.29476483e-02,-8.39032531e-01,-5.61413355e-04},
 {3.89224179e-02,-8.37963820e-01,-1.13316299e-03},
 {4.48913313e-02,-8.38659465e-01,-6.72814087e-04},
 {5.44416346e-02,-8.37041557e-01,4.50237468e-03},
 {6.19916841e-02,-8.33355367e-01,1.44633381e-02},
 {5.56045100e-02,-8.30426991e-01,7.25642173e-03},
 {4.58735041e-02,-8.29117715e-01,1.05240941e-03},
 {3.93655486e-02,-8.28876197e-01,3.16558842e-04},
 {3.28243151e-02,-8.29322457e-01,9.72221489e-04},
 {2.34481432e-02,-8.31013680e-01,7.00863171e-03},
 {1.74644589e-02,-8.34533095e-01,1.52696148e-02},
 {3.28513719e-02,-8.33426178e-01,4.90240380e-03},
 {3.89135554e-02,-8.33098233e-01,4.10967646e-03},
 {4.48884331e-02,-8.33178282e-01,4.67343628e-03},
 {6.17720485e-02,-8.33391428e-01,1.45598799e-02},
 {4.60876226e-02,-8.35628569e-01,4.25531063e-03},
 {3.96436155e-02,-8.35925281e-01,3.33748246e-03},
 {3.30116078e-02,-8.35752368e-01,4.15715668e-03}};

		int32_t m_ModelType;
		int32_t m_MaterialType;
		Eigen::Quaternionf m_ObjectRotation;
		bool m_Wireframe;

		int32_t m_Object_vis = 0; 

	};//ExampleShapesAndMaterials

}//namespace


#endif 