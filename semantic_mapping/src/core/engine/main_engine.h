#ifndef MAIN_ENGINE_H
#define MAIN_ENGINE_H

namespace semantic_mapping {
namespace main_engine {

class MainEngine {
public:

private:
  const Settings* settings_;
  bool fusion_active_, main_processing_active_;
  LowLevelEngine* low_level_engine_; 
  VisualisationEngine* visualisation_engine_;

};
		
} /* main_engine */ 		
} /* semantic_mapping */ 


#endif /* end of include guard: MAIN_ENGINE_H */
