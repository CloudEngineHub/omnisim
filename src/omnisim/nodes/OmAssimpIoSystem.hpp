// Copyright 2026 OmniLink
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OM_ASSIMP_IO_SYSTEM_HPP
#define OM_ASSIMP_IO_SYSTEM_HPP

//
// Description: Assimp file access that can open long Windows paths.
//
// Assimp's default IO system opens files with _wfopen on Windows, which cannot
// open a path longer than MAX_PATH (260 characters) unless the executable is
// long-path aware. This one hands _wfopen the absolute path with the
// "\\?\" long-path prefix, which can. Before this, a mesh in a deep folder (a URDF
// package unpacked a few levels down is enough) failed to import and the robot
// silently lost that geometry. Installing this handler on an Assimp::Importer
// routes every file it opens -- the mesh and any sibling it pulls in (an OBJ's
// .mtl, a glTF's .bin) -- through that path. No Qt: the engine core's Qt-include
// ratchet (tests/test_qt_include_ratchet.py) only goes down.
//

#include <assimp/IOStream.hpp>
#include <assimp/IOSystem.hpp>
#include <assimp/matrix4x4.h>

struct aiNode;

class OmAssimpIoSystem : public Assimp::IOSystem {
public:
  bool Exists(const char *file) const override;
  char getOsSeparator() const override { return '/'; }
  Assimp::IOStream *Open(const char *file, const char *mode = "rb") override;
  void Close(Assimp::IOStream *stream) override;
};

// The absolute (scene-root) transform of an Assimp node: root * ... * parent * node,
// the order that maps the node's mesh vertices into the scene frame. Shared by
// OmMesh and OmCadShape, which both walk the node graph.
//
// Both used to compose it leaf-first (node * parent * ... * root, inherited from
// Webots), which is the right answer only when at most one level carries a
// transform. Collada always has two: the importer's own up-axis rotation on the
// root, and any node translation that aiProcess_OptimizeGraph cannot bake away
// (it keeps the node of every mesh that is instanced more than once). Found
// 2026-10-15 on Rover Robotics' MEGA: mega_wheels.dae came out 26.5 mm off its
// axle (the base bounced +/-2.7 cm and the wheels stalled), mega_payload.dae
// up to 0.96 m out of place. OMNISIM_MESH_NODE_TRANSFORM_FIX=0 (value-parsed)
// restores the leaf-first product.
aiMatrix4x4 omAssimpNodeWorldTransform(const aiNode *node);

#endif
