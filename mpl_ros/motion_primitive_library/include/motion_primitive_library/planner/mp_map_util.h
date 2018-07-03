/**
 * @file mp_map_util.h
 * @brief motion planning map util
 */
#include <motion_primitive_library/planner/env_map.h>
#include <motion_primitive_library/planner/mp_base_util.h>

using linkedHashMap = std::unordered_map<int, std::vector<std::pair<MPL::Key, int>>>;
/**
 * @brief Motion primitive planner in voxel map
 */
template <int Dim>
class MPMapUtil : public MPBaseUtil<Dim>
{
  public:
    /**
     * @brief Simple constructor
     * @param verbose enable debug messages
     */
    MPMapUtil(bool verbose);
    ///Set map util
    void setMapUtil(std::shared_ptr<MPL::MapUtil<Dim>>& map_util);
    ///Get linked voxels
    vec_Vecf<Dim> getLinkedNodes() const;
    /**
     * @brief Update edge costs according to the new blocked nodes
     * @param pns the new occupied voxels 
     *
     * The function returns affected primitives for debug purpose
     */
    vec_E<Primitive<Dim>> updateBlockedNodes(const vec_Veci<Dim>& pns);
    /**
     * @brief Update edge costs according to the new cleared nodes
     * @param pns the new cleared voxels 
     *
     * The function returns affected primitives for debug purpose
     */
    vec_E<Primitive<Dim>> updateClearedNodes(const vec_Veci<Dim>& pns);

  protected:
    ///Map util
    std::shared_ptr<MPL::MapUtil<Dim>> map_util_;
    ///Linked table that records voxel and corresponding primitives passed through it
    mutable linkedHashMap lhm_;
};

///Planner for 2D OccMap
typedef MPMapUtil<2> MPMap2DUtil;

///Planner for 3D VoxelMap
typedef MPMapUtil<3> MPMap3DUtil;
