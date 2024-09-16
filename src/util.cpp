#include "common.hpp"
#include "util.hpp"

#ifdef _WIN32
#include "engine/kisslib/clean_windows_h.hpp"
#include <psapi.h>

void imgui_process_stats (bool* open) {
	if (!ImGui::Begin("Process Stats", open)) return;

	auto hproc = GetCurrentProcess();

	PROCESS_MEMORY_COUNTERS pmc = {};
	if (GetProcessMemoryInfo(hproc, &pmc, sizeof(pmc))) {
#define FMT(x) ((float)x / (1024.f*1024.f))

		ImGui::Text( "PageFaultCount:             %08llu",        pmc.PageFaultCount              );
		ImGui::Text( "PeakWorkingSetSize:         %08.3f MB", FMT(pmc.PeakWorkingSetSize        ) );
		ImGui::Text( "WorkingSetSize:             %08.3f MB", FMT(pmc.WorkingSetSize            ) );
		ImGui::Text( "QuotaPeakPagedPoolUsage:    %08.3f MB", FMT(pmc.QuotaPeakPagedPoolUsage   ) );
		ImGui::Text( "QuotaPagedPoolUsage:        %08.3f MB", FMT(pmc.QuotaPagedPoolUsage       ) );
		ImGui::Text( "QuotaPeakNonPagedPoolUsage: %08.3f MB", FMT(pmc.QuotaPeakNonPagedPoolUsage) );
		ImGui::Text( "QuotaNonPagedPoolUsage:     %08.3f MB", FMT(pmc.QuotaNonPagedPoolUsage    ) );
		ImGui::Text( "PagefileUsage:              %08.3f MB", FMT(pmc.PagefileUsage             ) ); 
		ImGui::Text( "PeakPagefileUsage:          %08.3f MB", FMT(pmc.PeakPagefileUsage         ) );

		// TODO: Cpu use?
	}

	ImGui::End();
}
#endif
