/*
 * tanoio - Tano IO node.js binding
 *
 * Author: Anton Kikin <a.kikin@tano-systems.com>
 *
 * Copyright (C) 2019 Tano Systems
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

var printf = require('printf');
var tanoio = require('..');

try {
	var tanoio_config = {
		url_ipc: "tcp://192.168.0.48:1111",
		url_events: "tcp://192.168.0.48:1112",
		events_callback: function(handle, event) {
			console.log({ handle: handle, event_data: event });
		}
	};

	var handle = tanoio.init(tanoio_config);
	printf(process.stdout, "Library initialized (handle = %d)\n", handle);

	var info = tanoio.get_info(handle);

	printf(process.stdout, "\nInformation:\n");
	printf(process.stdout, "- Version: %d.%d.%d\n", info.version_major, info.version_minor, info.version_fix);
	printf(process.stdout, "- Inputs count: %d\n", info.in_count);
	printf(process.stdout, "- Outputs count: %d\n", info.out_count);
	printf(process.stdout, "- Modules count: %d\n", info.modules_count);

	printf(process.stdout, "\nModules:\n");
	for (let i = 0; i < info.modules_count; i++)
	{
		var minfo = tanoio.module_get_info(handle, i);
		console.log(minfo);
	}

	printf(process.stdout, "\nInputs:\n");
	printf(process.stdout, "-------------------------------------------------------------------------------\n");
	printf(process.stdout, "| %-3s | %-3s | %-8s | %-30s | %-6s | %-10s |\n",
		"ID", "MID", "Name", "Description", "Type", "Features");
	printf(process.stdout, "-------------------------------------------------------------------------------\n");

	for (let i = 0; i < info.in_count; i++)
	{
		var iinfo = tanoio.in_get_info(handle, i);

		printf(process.stdout, "| %3u | %3u | %-8s | %-30s | %-6d | 0x%08x |\n",
			iinfo.id,
			iinfo.module_id,
			iinfo.name,
			iinfo.description,
			iinfo.type,
			iinfo.features_u32
		);
	}

	printf(process.stdout, "-------------------------------------------------------------------------------\n");

	printf(process.stdout, "\nOutputs:\n");
	printf(process.stdout, "-------------------------------------------------------------------------------\n");
	printf(process.stdout, "| %-3s | %-3s | %-8s | %-30s | %-6s | %-10s |\n",
		"ID", "MID", "Name", "Description", "Type", "Features");
	printf(process.stdout, "-------------------------------------------------------------------------------\n");

	for (let i = 0; i < info.out_count; i++)
	{
		var oinfo = tanoio.out_get_info(handle, i);

		printf(process.stdout, "| %3u | %3u | %-8s | %-30s | %-6d | 0x%08x |\n",
			oinfo.id,
			oinfo.module_id,
			oinfo.name,
			oinfo.description,
			oinfo.type,
			oinfo.features_u32
		);
	}

	printf(process.stdout, "-------------------------------------------------------------------------------\n");

	// Wait
	printf(process.stdout, "\nListening for events (press Ctrl+C for exit)...\n");

	var set_mask = 0xff;
	var values_mask = 0x55;

	tanoio.out_set_mode(handle, 0, tanoio.TANOIO_OUT_MODE_DIRECT);
	tanoio.out_set_mode(handle, 1, tanoio.TANOIO_OUT_MODE_DIRECT);
	tanoio.out_set_mode(handle, 2, tanoio.TANOIO_OUT_MODE_DIRECT);
	tanoio.out_set_mode(handle, 3, tanoio.TANOIO_OUT_MODE_DIRECT);
	tanoio.out_set_mode(handle, 4, tanoio.TANOIO_OUT_MODE_DIRECT);
	tanoio.out_set_mode(handle, 5, tanoio.TANOIO_OUT_MODE_DIRECT);
	tanoio.out_set_mode(handle, 6, tanoio.TANOIO_OUT_MODE_DIRECT);
	tanoio.out_set_mode(handle, 7, tanoio.TANOIO_OUT_MODE_DIRECT);

	tanoio.destroy(handle);
	printf(process.stdout, "Library destroyed\n");

	handle = tanoio.init(tanoio_config);
	printf(process.stdout, "Library initialized (handle = %d)\n", handle);

	setInterval(function() {
		tanoio.out_set_states_mask(handle, set_mask, values_mask);
		tanoio.out_set_states_mask(handle, set_mask, values_mask);
		values_mask = ~values_mask;
	}, 1000);

} catch(error) {
	console.log(error);
}
