# pragma once

#include <vector>
#include <png.h>
#include <crossforge/Utility/CForgeUtility.h>
#include <crossforge/Math/CForgeMath.h>

namespace CForge
{
    class PNGImport{
    public:
        // PNGImport(void);
        // ~PNGImport(void);
        
        void readPNGFile(const std::string &fileName); 
        Eigen::Vector4d getPixel(int x, int y); 
        Eigen::Vector4d uvCoordColor(float x, float y);
        bool isPictureLoaded(void);
        Eigen::Vector4d getMeanColor(T3DMesh<float>*pMesh, int submeshIndex ,std::vector<int> *faceIndices, T3DMesh<float> *pOrigin, std::vector<int32_t> *correspondencyNotInverted);
    
    protected:
        int m_height; 
        int m_width;

        std::vector<std::vector<Eigen::Vector4d>> m_pixelValues;  
    };

    class ColorSpace{
    public:
        static double differenceRGBValues(Eigen::Vector3f rgb1, Eigen::Vector3f rgb2);
    };

    
} // namespace CForge