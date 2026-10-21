#pragma once

namespace vsim {

const char* version();
// Short commit hash of the source tree at configure time ("unknown" outside git).
const char* git_commit();

}  // namespace vsim
