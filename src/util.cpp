#include "common.hpp"
#include "util.hpp"

#ifdef _WIN32
#include "engine/kisslib/clean_windows_h.hpp"
#include <psapi.h>

size_t imgui_process_stats () {
	auto hproc = GetCurrentProcess();

	PROCESS_MEMORY_COUNTERS pmc = {};
	if (GetProcessMemoryInfo(hproc, &pmc, sizeof(pmc)) &&
		ImGui::TreeNodeEx("OS Process Stats", ImGuiTreeNodeFlags_DefaultOpen)) {
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

		ImGui::TreePop();
		return pmc.WorkingSetSize;
	}

	return 0;
}
#endif

void MemUse::_imgui () {
	ImGui::HelpMarker("=== Memory Usage Tracking ===\n"
	"Explicitly counts up some allocated memory\n"
	"Does NOT count:\n"
	" -Any untracked data\n"
	" -Heap allocation overhead\n"
	" -Potential Memory leaks\n");

	std::vector<Name> ordered;
	for (auto kv : sizes)
		ordered.push_back(kv.first);
	std::stable_sort(ordered.begin(), ordered.end());
		
	size_t total = 0;

	auto color = [] (size_t sz) {
		if      (sz > 1024ull*1024*1024) return ImColor(1.00f,0.25f,0.75f);
		else if (sz > 1024ull*1024     ) return ImColor(1.00f,0.50f,0.25f);
		else if (sz > 1024ull          ) return ImColor(0.75f,0.65f,0.15f);
		else                             return ImColor(0.50f,0.50f,0.55f);
	};

	for (auto key : ordered) {
		auto& entry = sizes[key];

		ImGui::TextColored(color(entry.size), "%25s:%7llu %10s", key, entry.count, format_bytes(entry.size).c_str());

		total += entry.size;
	}
	
	ImGui::Separator();
	ImGui::TextColored(color(total), "Total Tracked: %10s", format_bytes(total).c_str());

	size_t used_ram = imgui_process_stats();

	ImGui::Separator();
	ImGui::TextColored(color(used_ram), "Total Reported by System: %10s (%.2f %% Tracked)",
		format_bytes(used_ram).c_str(), (float)total / (float)used_ram * 100);
}
