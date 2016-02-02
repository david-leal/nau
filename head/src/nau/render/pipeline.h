#ifndef PIPELINE_H
#define PIPELINE_H


#include "nau/material/iState.h"
#include "nau/scene/camera.h"
#include "nau/scene/sceneObject.h"
#include "nau/render/pass.h"

#include <deque>
#include <fstream>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <string>

#define PIPE_PASS_MIDDLE 0
#define PIPE_PASS_START 1
#define PIPE_PASS_STARTEND 2
#define PIPE_PASS_END 3

namespace nau
{
	namespace render
	{

		class Pipeline {

			friend class RenderManager;
		public:

			~Pipeline();

			std::string getName();
			void getPassNames(std::vector<std::string> *);

			int getNumberOfPasses();
			int getPassCounter();

			/** 
			 * Add a pass to the pipeline. 
			 * 
			 * \param aPass The pass object to add
			 * \param PassIndex The pipeline position to insert the pass. 
			 *                  0 is the first pass. -1 is the last. 
			 */
			//void addPass (Pass* aPass, int PassIndex = -1);
			Pass* createPass (const std::string &name, const std::string &passName = "default");

			bool hasPass(const std::string &passName);
			Pass* getPass (const std::string &passName);
			Pass* getPass (int n);
			Pass *getCurrentPass();

			void setFrameCount(unsigned int k);
			unsigned int getFrameCount();

			//! Gets the name of the camera from the current pass being executed
			const std::string &getCurrentCamera();
			const std::string &getLastPassCameraName(); 

			//! Gets the default camera, if not set it returns the last pass camera
			const std::string &getDefaultCameraName();
			void setDefaultCamera(const std::string &defCam);

			void initState(IState *state);


			void execute();

			void executeNextPass();

		
			// -----------------------------------------------------------------
			//		PRE POST SCRIPTS
			// -----------------------------------------------------------------
			void setPreScript(std::string file, std::string name);
			void setPostScript(std::string file, std::string name);
			void callScript(std::string &name);

		protected:

			Pipeline(std::string pipelineName = "Default");

			void executePass(std::shared_ptr<Pass> &);
			Pipeline (const Pipeline&);
			Pipeline& operator= (const Pipeline&);

			std::deque<std::shared_ptr<Pass>> m_Passes;
			std::string m_Name;

			//! The default camera will receive events from the EventManager
			std::string m_DefaultCamera;

			std::shared_ptr<Pass> m_CurrentPass;

			unsigned int m_NextPass;

			std::string m_PreScriptFile, m_PreScriptName,
				m_PostScriptFile, m_PostScriptName;

			unsigned int m_FrameCount = 0;
		};
	};
};

#endif

