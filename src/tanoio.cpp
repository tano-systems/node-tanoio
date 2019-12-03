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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <tanoio.h>

#include "nan.h"

using namespace node;
using namespace v8;

static void toCString(Local<String> val, char **ptr)
{
	*ptr = new char[val->Utf8Length() + 1];
#if (NODE_MODULE_VERSION > 0x000B)
	// Node 0.11+ (0.11.3 and below won't compile with these)
	val->WriteOneByte(reinterpret_cast<uint8_t*>(*ptr), 0, -1, 0);
#else
	// Node 0.8 and 0.10
	val->WriteAscii(*ptr, 0, -1, 0);
#endif
}

static inline v8::Local<v8::Value> makeTanoIoError(
	const char *func,
	const char *msg,
	tanoio_status_t status
)
{
	Nan::EscapableHandleScope scope;
	v8::Local<v8::Object> err = Nan::Error("TanoIO error").As<v8::Object>();
	err->Set(Nan::New("status").ToLocalChecked(), Nan::New<Number>(status));

	if (func)
		err->Set(Nan::New("function").ToLocalChecked(),
			Nan::New<String>(func).ToLocalChecked());

	if (msg)
		err->Set(Nan::New("msg").ToLocalChecked(),
			Nan::New<String>(msg).ToLocalChecked());

	return scope.Escape(err);
}

#define throw_tanoio_error(status, msg) \
	Nan::ThrowError(makeTanoIoError(__FUNCTION__ , msg, status));

v8::Persistent<v8::Function> tanoio_events_callback;

typedef struct
{
	tanoio_handle_t handle;
	struct tanoio_event *event;

} native_event_data_t;

static void event_handler_w(uv_work_t* req)
{
	/* Nothing */
}

static void event_handler_a(uv_work_t* req, int arg)
{
	Isolate *isolate = v8::Isolate::GetCurrent();
	HandleScope scope(isolate);

	native_event_data_t *data =
		(native_event_data_t *)req->data;

	Local<Object> event_obj = Nan::New<v8::Object>();

	Nan::Set(event_obj, Nan::New("id").ToLocalChecked(),
		Nan::New<v8::Uint32>(data->event->id));

	Nan::Set(event_obj, Nan::New("io_id").ToLocalChecked(),
		Nan::New<v8::Uint32>(data->event->io_id));

	Nan::Set(event_obj, Nan::New("ts_sec").ToLocalChecked(),
		Nan::New<v8::Uint32>(data->event->ts.sec));

	Nan::Set(event_obj, Nan::New("ts_nsec").ToLocalChecked(),
		Nan::New<v8::Uint32>(data->event->ts.nsec));

	switch(data->event->id)
	{
		case TANOIO_EVENT_IN_STATE_CHANGED:
		{
			struct tanoio_event_data_in_state_changed *ev_data =
				(struct tanoio_event_data_in_state_changed *)data->event->data;

			Nan::Set(event_obj, Nan::New("state").ToLocalChecked(),
				Nan::New<v8::Uint32>(ev_data->state));

			Nan::Set(event_obj, Nan::New("counter").ToLocalChecked(),
				Nan::New<v8::Uint32>((unsigned int)ev_data->counter));

			break;
		}

		case TANOIO_EVENT_OUT_ACCIDENT:
		{
			struct tanoio_event_data_out_accident *ev_data =
				(struct tanoio_event_data_out_accident *)data->event->data;

			Nan::Set(event_obj, Nan::New("type").ToLocalChecked(),
				Nan::New<v8::Uint32>(ev_data->type));

			Nan::Set(event_obj, Nan::New("counter").ToLocalChecked(),
				Nan::New<v8::Uint32>((unsigned int)ev_data->counter));

			break;
		}

		default:
			break;
	}

	v8::Local<v8::Value> argv[] =
	{
		Nan::New(data->handle),
		event_obj
	};

	delete[] data->event;
	delete data;

	v8::Local<v8::Function> lf =
		v8::Local<v8::Function>::New(isolate, tanoio_events_callback);

	lf->Call(Null(isolate), 2, argv);
}

static void native_events_callback(
	tanoio_handle_t handle,
	const struct tanoio_event *event)
{
	native_event_data_t *data = new native_event_data_t;

	data->handle = handle;
	data->event = (struct tanoio_event *)new char[sizeof(struct tanoio_event) +
		event->data_size];

	memcpy(data->event, event,
		sizeof(struct tanoio_event) + event->data_size);

	uv_work_t* req = new uv_work_t();
	req->data = data;

	uv_queue_work(uv_default_loop(), req,
		event_handler_w, event_handler_a);
}

#define OBJECT_SET_U32(obj, struct, field) \
	Nan::Set(obj, Nan::New(# field).ToLocalChecked(), \
		Nan::New<v8::Uint32>(struct.field))

#define OBJECT_SET_BOOL(obj, struct, field) \
	Nan::Set(obj, Nan::New(# field).ToLocalChecked(), \
		Nan::New<v8::Boolean>(struct.field))

#define OBJECT_SET_STRING(obj, struct, field) \
	Nan::Set(obj, Nan::New(# field).ToLocalChecked(), \
		Nan::New<v8::String>(struct.field).ToLocalChecked())

#define TANOIO_CALL_FUNC_DEF_GET(func, c_type, js_type) \
	Nan::EscapableHandleScope scope; \
	\
	tanoio_handle_t handle = Nan::To<int>(info[0]).FromJust(); \
	unsigned int id = Nan::To<unsigned int>(info[1]).FromJust(); \
	c_type value; \
	\
	tanoio_status_t status; \
	\
	status = func(handle, id, &value); \
	if (status != TANOIO_E_OK) \
		throw_tanoio_error(status, NULL); \
	\
	info.GetReturnValue().Set(Nan::New<js_type>(value))

#define TANOIO_CALL_FUNC_DEF_SET(func, c_type, js_type) \
	Nan::EscapableHandleScope scope; \
	\
	tanoio_handle_t handle = Nan::To<int>(info[0]).FromJust(); \
	unsigned int id = Nan::To<unsigned int>(info[1]).FromJust(); \
	c_type value = (c_type)Nan::To<unsigned int>(info[2]).FromJust(); \
	\
	tanoio_status_t status; \
	\
	status = func(handle, id, value); \
	info.GetReturnValue().Set(Nan::New<v8::Number>(status))

#define TANOIO_CALL_FUNC_DEF(func) \
	Nan::EscapableHandleScope scope; \
	\
	tanoio_handle_t handle = Nan::To<int>(info[0]).FromJust(); \
	unsigned int id = Nan::To<unsigned int>(info[1]).FromJust(); \
	\
	tanoio_status_t status; \
	\
	status = func(handle, id); \
	info.GetReturnValue().Set(Nan::New<v8::Number>(status))

class TanoIO
{
	public:

	/*
	 *   _____            _             _
	 *  / ____|          | |           | |
	 * | |     ___  _ __ | |_ _ __ ___ | |
	 * | |    / _ \| '_ \| __| '__/ _ \| |
	 * | |___| (_) | | | | |_| | | (_) | |
	 *  \_____\___/|_| |_|\__|_|  \___/|_|
	 *
	 */
	static NAN_METHOD(init)
	{
		Nan::EscapableHandleScope scope;

		// Configuration
		Local<Object> config_obj = info[0].As<v8::Object>();

		tanoio_handle_t handle;
		tanoio_status_t status;
		struct tanoio_config config;

		toCString(Nan::Get(config_obj, Nan::New("url_ipc")
			.ToLocalChecked()).ToLocalChecked()->ToString(), &config.url_ipc);

		toCString(Nan::Get(config_obj, Nan::New("url_events")
			.ToLocalChecked()).ToLocalChecked()->ToString(), &config.url_events);

		// Setup events callback function
		config.events_callback = native_events_callback;

		auto cb_func = v8::Local<v8::Function>::Cast(
			config_obj->Get(Nan::New<String>("events_callback").ToLocalChecked()));

		tanoio_events_callback.Reset(v8::Isolate::GetCurrent(), cb_func);

		// Initialize library
		status = tanoio_init(&config, &handle);

		free(config.url_ipc);
		free(config.url_events);

		if (status != TANOIO_E_OK)
			throw_tanoio_error(status, NULL);

		info.GetReturnValue().Set(Nan::New<v8::Number>(handle));
	}

	static NAN_METHOD(destroy)
	{
		Nan::EscapableHandleScope scope;

		tanoio_handle_t handle = Nan::To<int>(info[0]).FromJust();
		tanoio_destroy(handle);
		info.GetReturnValue().SetUndefined();
	}

	/*
	 *  _____ _       _           _
	 * / ____| |     | |         | |
	 *| |  __| | ___ | |__   __ _| |
	 *| | |_ | |/ _ \| '_ \ / _` | |
	 *| |__| | | (_) | |_) | (_| | |
	 * \_____|_|\___/|_.__/ \__,_|_|
	 *
	 */
	static NAN_METHOD(get_info)
	{
		Nan::EscapableHandleScope scope;

		// Return object
		Local<Object> return_obj = Nan::New<v8::Object>();

		tanoio_handle_t handle = Nan::To<int>(info[0]).FromJust();

		tanoio_status_t status;
		struct tanoio_info tinfo;

		status = tanoio_get_info(handle, &tinfo);

		if (status != TANOIO_E_OK)
			throw_tanoio_error(status, NULL);

		Nan::Set(return_obj, Nan::New("version_major").ToLocalChecked(),
			Nan::New<v8::Uint32>(TANOIO_VERSION_GET_MAJOR(tinfo.version)));

		Nan::Set(return_obj, Nan::New("version_minor").ToLocalChecked(),
			Nan::New<v8::Uint32>(TANOIO_VERSION_GET_MINOR(tinfo.version)));

		Nan::Set(return_obj, Nan::New("version_fix").ToLocalChecked(),
			Nan::New<v8::Uint32>(TANOIO_VERSION_GET_FIX(tinfo.version)));

		OBJECT_SET_U32(return_obj, tinfo, in_count);
		OBJECT_SET_U32(return_obj, tinfo, out_count);
		OBJECT_SET_U32(return_obj, tinfo, modules_count);

		info.GetReturnValue().Set(scope.Escape(return_obj));
	}

	/*
	 *  __  __           _       _
	 * |  \/  |         | |     | |
	 * | \  / | ___   __| |_   _| | ___  ___
	 * | |\/| |/ _ \ / _` | | | | |/ _ \/ __|
	 * | |  | | (_) | (_| | |_| | |  __/\__ \
	 * |_|  |_|\___/ \__,_|\__,_|_|\___||___/
	 *
	 */
	static NAN_METHOD(module_get_info)
	{
		Nan::EscapableHandleScope scope;

		// Return object
		Local<Object> return_obj = Nan::New<v8::Object>();
		Local<Object> features_obj = Nan::New<v8::Object>();

		tanoio_handle_t handle = Nan::To<int>(info[0]).FromJust();
		unsigned int id = Nan::To<unsigned int>(info[1]).FromJust();

		tanoio_status_t status;
		struct tanoio_module_info minfo;

		status = tanoio_module_get_info(handle, id, &minfo);
		if (status != TANOIO_E_OK)
			throw_tanoio_error(status, NULL);

		Nan::Set(features_obj, Nan::New("in_states_mask").ToLocalChecked(),
			Nan::New<v8::Boolean>(!!(minfo.features & TANOIO_MODULE_FT_IN_STATES_MASK)));

		Nan::Set(features_obj, Nan::New("out_states_mask").ToLocalChecked(),
			Nan::New<v8::Boolean>(!!(minfo.features & TANOIO_MODULE_FT_OUT_STATES_MASK)));

		Nan::Set(features_obj, Nan::New("out_safe_timeout").ToLocalChecked(),
			Nan::New<v8::Boolean>(!!(minfo.features & TANOIO_MODULE_FT_OUT_SAFE_TIMEOUT)));

		Nan::Set(features_obj, Nan::New("out_safe_states_mask").ToLocalChecked(),
			Nan::New<v8::Boolean>(!!(minfo.features & TANOIO_MODULE_FT_OUT_SAFE_STATES_MASK)));

		Nan::Set(features_obj, Nan::New("manual_sync").ToLocalChecked(),
			Nan::New<v8::Boolean>(!!(minfo.features & TANOIO_MODULE_FT_MANUAL_SYNC)));

		OBJECT_SET_U32(return_obj, minfo, id);

		Nan::Set(return_obj, Nan::New("features_u32").ToLocalChecked(),
			Nan::New<v8::Uint32>(minfo.features));

		Nan::Set(return_obj, Nan::New("features").ToLocalChecked(), features_obj);

		OBJECT_SET_U32(return_obj, minfo, in_count);
		OBJECT_SET_U32(return_obj, minfo, in_start_id);
		OBJECT_SET_U32(return_obj, minfo, out_count);
		OBJECT_SET_U32(return_obj, minfo, out_start_id);
		OBJECT_SET_U32(return_obj, minfo, out_count);

		Nan::Set(return_obj, Nan::New("version_major").ToLocalChecked(),
			Nan::New<v8::Uint32>(TANOIO_VERSION_GET_MAJOR(minfo.version)));

		Nan::Set(return_obj, Nan::New("version_minor").ToLocalChecked(),
			Nan::New<v8::Uint32>(TANOIO_VERSION_GET_MINOR(minfo.version)));

		Nan::Set(return_obj, Nan::New("version_fix").ToLocalChecked(),
			Nan::New<v8::Uint32>(TANOIO_VERSION_GET_FIX(minfo.version)));

		OBJECT_SET_STRING(return_obj, minfo, name);
		OBJECT_SET_STRING(return_obj, minfo, description);
		OBJECT_SET_STRING(return_obj, minfo, hardware_id);
		OBJECT_SET_STRING(return_obj, minfo, requires);
		OBJECT_SET_STRING(return_obj, minfo, io_after);
		OBJECT_SET_STRING(return_obj, minfo, io_before);

		info.GetReturnValue().Set(scope.Escape(return_obj));
	}

	static NAN_METHOD(module_get_params)
	{
		Nan::EscapableHandleScope scope;

		// Return object
		Local<Object> return_obj = Nan::New<v8::Object>();

		tanoio_handle_t handle = Nan::To<int>(info[0]).FromJust();
		unsigned int id = Nan::To<unsigned int>(info[1]).FromJust();

		tanoio_status_t status;
		struct tanoio_module_info minfo;
		struct tanoio_module_params mparams;

		status = tanoio_module_get_info(handle, id, &minfo);
		if (status != TANOIO_E_OK)
			throw_tanoio_error(status, NULL);

		status = tanoio_module_get_params(handle, id, &mparams);
		if (status != TANOIO_E_OK)
			throw_tanoio_error(status, NULL);

		OBJECT_SET_U32(return_obj, minfo, id);

		if (minfo.features & TANOIO_MODULE_FT_OUT_SAFE_TIMEOUT)
			OBJECT_SET_U32(return_obj, mparams, out_safe_timeout_ms);

		if (minfo.features & TANOIO_MODULE_FT_MANUAL_SYNC)
		{
			Nan::Set(return_obj, Nan::New("manual_sync").ToLocalChecked(),
				Nan::New<v8::Boolean>(!!(mparams.flags & TANOIO_MODULE_FL_MANUAL_SYNC)));
		}

		info.GetReturnValue().Set(scope.Escape(return_obj));
	}

	static NAN_METHOD(module_out_set_safe_timeout)
	{
		TANOIO_CALL_FUNC_DEF_SET(
			tanoio_module_out_set_safe_timeout,
			unsigned int, v8::Number);
	}

	static NAN_METHOD(module_out_get_safe_timeout)
	{
		TANOIO_CALL_FUNC_DEF_GET(
			tanoio_module_out_get_safe_timeout,
			unsigned int, v8::Number);
	}

	static NAN_METHOD(module_get_manual_sync)
	{
		TANOIO_CALL_FUNC_DEF_GET(
			tanoio_module_get_manual_sync,
			tanoio_bool_t, v8::Boolean);
	}

	static NAN_METHOD(module_set_manual_sync)
	{
		TANOIO_CALL_FUNC_DEF_SET(
			tanoio_module_set_manual_sync,
			tanoio_bool_t, v8::Boolean);
	}

	static NAN_METHOD(module_sync)
	{
		TANOIO_CALL_FUNC_DEF(tanoio_module_sync);
	}

	/*
	 *  _____                   _
	 * |_   _|                 | |
	 *   | |  _ __  _ __  _   _| |_ ___
	 *   | | | '_ \| '_ \| | | | __/ __|
	 *  _| |_| | | | |_) | |_| | |_\__ \
	 * |_____|_| |_| .__/ \__,_|\__|___/
	 *             | |
	 *             |_|
	 */

	static NAN_METHOD(in_get_info)
	{
		Nan::EscapableHandleScope scope;

		// Return object
		Local<Object> return_obj = Nan::New<v8::Object>();
		Local<Object> features_obj = Nan::New<v8::Object>();

		tanoio_handle_t handle = Nan::To<int>(info[0]).FromJust();
		unsigned int id = Nan::To<unsigned int>(info[1]).FromJust();

		tanoio_status_t status;
		struct tanoio_in_info iinfo;

		status = tanoio_in_get_info(handle, id, &iinfo);
		if (status != TANOIO_E_OK)
			throw_tanoio_error(status, NULL);

		OBJECT_SET_STRING(return_obj, iinfo, name);
		OBJECT_SET_STRING(return_obj, iinfo, description);

		OBJECT_SET_U32(return_obj, iinfo, id);
		OBJECT_SET_U32(return_obj, iinfo, module_id);
		OBJECT_SET_U32(return_obj, iinfo, index);

		Nan::Set(features_obj, Nan::New("debounce").ToLocalChecked(),
			Nan::New<v8::Boolean>(!!(iinfo.features & TANOIO_IN_FT_DEBOUNCE)));

		Nan::Set(features_obj, Nan::New("reset").ToLocalChecked(),
			Nan::New<v8::Boolean>(!!(iinfo.features & TANOIO_IN_FT_RESET)));

		Nan::Set(features_obj, Nan::New("filter").ToLocalChecked(),
			Nan::New<v8::Boolean>(!!(iinfo.features & TANOIO_IN_FT_FILTER)));

		Nan::Set(features_obj, Nan::New("active_state").ToLocalChecked(),
			Nan::New<v8::Boolean>(!!(iinfo.features & TANOIO_IN_FT_ACTIVE_STATE)));

		Nan::Set(features_obj, Nan::New("edge").ToLocalChecked(),
			Nan::New<v8::Boolean>(!!(iinfo.features & TANOIO_IN_FT_EDGE)));

		Nan::Set(features_obj, Nan::New("mode_counter").ToLocalChecked(),
			Nan::New<v8::Boolean>(!!(iinfo.features & TANOIO_IN_FT_MODE_COUNTER)));

		Nan::Set(features_obj, Nan::New("mode_ab_encoder").ToLocalChecked(),
			Nan::New<v8::Boolean>(!!(iinfo.features & TANOIO_IN_FT_MODE_AB_ENCODER)));

		Nan::Set(features_obj, Nan::New("mode_abz_encoder").ToLocalChecked(),
			Nan::New<v8::Boolean>(!!(iinfo.features & TANOIO_IN_FT_MODE_ABZ_ENCODER)));

		Nan::Set(features_obj, Nan::New("mode_duty_ratio").ToLocalChecked(),
			Nan::New<v8::Boolean>(!!(iinfo.features & TANOIO_IN_FT_MODE_DUTY_RATIO)));

		Nan::Set(return_obj, Nan::New("features_u32").ToLocalChecked(),
			Nan::New<v8::Uint32>(iinfo.features));

		Nan::Set(return_obj, Nan::New("features").ToLocalChecked(), features_obj);

		OBJECT_SET_U32(return_obj, iinfo, type);

		info.GetReturnValue().Set(scope.Escape(return_obj));
	}

	static NAN_METHOD(in_get_params)
	{
		Nan::EscapableHandleScope scope;

		// Return object
		Local<Object> return_obj = Nan::New<v8::Object>();

		tanoio_handle_t handle = Nan::To<int>(info[0]).FromJust();
		unsigned int id = Nan::To<unsigned int>(info[1]).FromJust();

		tanoio_status_t status;
		struct tanoio_in_info iinfo;
		struct tanoio_in_params iparams;

		status = tanoio_in_get_info(handle, id, &iinfo);
		if (status != TANOIO_E_OK)
			throw_tanoio_error(status, NULL);

		status = tanoio_in_get_params(handle, id, &iparams);
		if (status != TANOIO_E_OK)
			throw_tanoio_error(status, NULL);

		OBJECT_SET_U32(return_obj, iinfo, id);

		OBJECT_SET_U32(return_obj, iparams, mode);
		OBJECT_SET_BOOL(return_obj, iparams, state);

		if (iinfo.features & TANOIO_IN_FT_DEBOUNCE)
		{
			Nan::Set(return_obj, Nan::New("debounce").ToLocalChecked(),
				Nan::New<v8::Boolean>(!!(iparams.flags & TANOIO_IN_FL_DEBOUNCE)));
		}

		if ((iparams.mode == TANOIO_IN_MODE_COUNTER) ||
		    (iparams.mode == TANOIO_IN_MODE_ABZ_ENCODER))
			Nan::Set(return_obj, Nan::New("counter").ToLocalChecked(),
				Nan::New<v8::Uint32>((unsigned int)iparams.counter));

		if (iparams.mode == TANOIO_IN_MODE_DUTY_RATIO)
		{
			OBJECT_SET_U32(return_obj, iparams, measured_duty_us);
			OBJECT_SET_U32(return_obj, iparams, measured_period_us);
		}

		if ((iparams.mode == TANOIO_IN_MODE_AB_ENCODER) ||
		    (iparams.mode == TANOIO_IN_MODE_ABZ_ENCODER))
		{
			OBJECT_SET_U32(return_obj, iparams, ab_encoder);
		}

		if (iparams.mode == TANOIO_IN_MODE_ABZ_ENCODER)
		{
			OBJECT_SET_U32(return_obj, iparams, abz_encoder);
		}

		info.GetReturnValue().Set(scope.Escape(return_obj));
	}

	static NAN_METHOD(in_get_states_mask)
	{
		Nan::EscapableHandleScope scope;

		tanoio_handle_t handle = Nan::To<int>(info[0]).FromJust();
		tanoio_mask_t value;

		tanoio_status_t status;

		status = tanoio_in_get_states_mask(handle, &value);
		if (status != TANOIO_E_OK)
			throw_tanoio_error(status, NULL);

		info.GetReturnValue().Set(Nan::New<v8::Number>(value));
	}

	static NAN_METHOD(in_get_mode)
	{
		TANOIO_CALL_FUNC_DEF_GET(
			tanoio_in_get_mode,
			tanoio_in_mode_t, v8::Number);
	}

	static NAN_METHOD(in_set_mode)
	{
		TANOIO_CALL_FUNC_DEF_SET(
			tanoio_in_set_mode,
			tanoio_in_mode_t, v8::Number);
	}

	static NAN_METHOD(in_get_counter)
	{
		TANOIO_CALL_FUNC_DEF_GET(
			tanoio_in_get_counter,
			unsigned long long, v8::Number);
	}

	static NAN_METHOD(in_reset)
	{
		TANOIO_CALL_FUNC_DEF(tanoio_in_reset);
	}

	static NAN_METHOD(in_get_state)
	{
		TANOIO_CALL_FUNC_DEF_GET(
			tanoio_in_get_state,
			tanoio_state_t, v8::Boolean);
	}

	static NAN_METHOD(in_get_debounce)
	{
		TANOIO_CALL_FUNC_DEF_GET(
			tanoio_in_get_debounce,
			tanoio_bool_t, v8::Boolean);
	}

	static NAN_METHOD(in_set_debounce)
	{
		TANOIO_CALL_FUNC_DEF_SET(
			tanoio_in_set_debounce,
			tanoio_bool_t, v8::Boolean);
	}

	static NAN_METHOD(in_set_filter_period)
	{
		TANOIO_CALL_FUNC_DEF_SET(
			tanoio_in_set_filter_period,
			unsigned int, v8::Number);
	}

	static NAN_METHOD(in_get_filter_period)
	{
		TANOIO_CALL_FUNC_DEF_GET(
			tanoio_in_get_filter_period,
			unsigned int, v8::Number);
	}

	static NAN_METHOD(in_set_filter_length)
	{
		TANOIO_CALL_FUNC_DEF_SET(
			tanoio_in_set_filter_length,
			unsigned int, v8::Number);
	}

	static NAN_METHOD(in_get_filter_length)
	{
		TANOIO_CALL_FUNC_DEF_GET(
			tanoio_in_get_filter_length,
			unsigned int, v8::Number);
	}

	static NAN_METHOD(in_get_measured_period)
	{
		TANOIO_CALL_FUNC_DEF_GET(
			tanoio_in_get_measured_period,
			unsigned int, v8::Number);
	}

	static NAN_METHOD(in_get_measured_duty)
	{
		TANOIO_CALL_FUNC_DEF_GET(
			tanoio_in_get_measured_duty,
			unsigned int, v8::Number);
	}

	static NAN_METHOD(in_get_ab_encoder)
	{
		TANOIO_CALL_FUNC_DEF_GET(
			tanoio_in_get_ab_encoder,
			int, v8::Number);
	}

	static NAN_METHOD(in_get_abz_encoder)
	{
		TANOIO_CALL_FUNC_DEF_GET(
			tanoio_in_get_abz_encoder,
			int, v8::Number);
	}

	static NAN_METHOD(in_set_active_state)
	{
		TANOIO_CALL_FUNC_DEF_SET(
			tanoio_in_set_active_state,
			tanoio_active_state_t, v8::Boolean);
	}

	static NAN_METHOD(in_get_active_state)
	{
		TANOIO_CALL_FUNC_DEF_GET(
			tanoio_in_get_active_state,
			tanoio_active_state_t, v8::Boolean);
	}

	static NAN_METHOD(in_set_edge)
	{
		TANOIO_CALL_FUNC_DEF_SET(
			tanoio_in_set_edge,
			tanoio_edge_t, v8::Number);
	}

	static NAN_METHOD(in_get_edge)
	{
		TANOIO_CALL_FUNC_DEF_GET(
			tanoio_in_get_edge,
			tanoio_edge_t, v8::Number);
	}

	/*
	 *   ____        _               _
	 *  / __ \      | |             | |
	 * | |  | |_   _| |_ _ __  _   _| |_ ___
	 * | |  | | | | | __| '_ \| | | | __/ __|
	 * | |__| | |_| | |_| |_) | |_| | |_\__ \
	 *  \____/ \__,_|\__| .__/ \__,_|\__|___/
	 *                  | |
	 *                  |_|
	 */
	static NAN_METHOD(out_get_info)
	{
		Nan::EscapableHandleScope scope;

		// Return object
		Local<Object> return_obj = Nan::New<v8::Object>();
		Local<Object> features_obj = Nan::New<v8::Object>();

		tanoio_handle_t handle = Nan::To<int>(info[0]).FromJust();
		unsigned int id = Nan::To<unsigned int>(info[1]).FromJust();

		tanoio_status_t status;
		struct tanoio_out_info oinfo;

		status = tanoio_out_get_info(handle, id, &oinfo);
		if (status != TANOIO_E_OK)
			throw_tanoio_error(status, NULL);

		OBJECT_SET_STRING(return_obj, oinfo, name);
		OBJECT_SET_STRING(return_obj, oinfo, description);

		OBJECT_SET_U32(return_obj, oinfo, id);
		OBJECT_SET_U32(return_obj, oinfo, module_id);
		OBJECT_SET_U32(return_obj, oinfo, index);

		Nan::Set(features_obj, Nan::New("accidents").ToLocalChecked(),
			Nan::New<v8::Boolean>(!!(oinfo.features & TANOIO_OUT_FT_ACCIDENTS)));

		Nan::Set(features_obj, Nan::New("accidents_count").ToLocalChecked(),
			Nan::New<v8::Boolean>(!!(oinfo.features & TANOIO_OUT_FT_ACCIDENTS_COUNT)));

		Nan::Set(features_obj, Nan::New("accident_type").ToLocalChecked(),
			Nan::New<v8::Boolean>(!!(oinfo.features & TANOIO_OUT_FT_ACCIDENT_TYPE)));

		Nan::Set(features_obj, Nan::New("conn_type").ToLocalChecked(),
			Nan::New<v8::Boolean>(!!(oinfo.features & TANOIO_OUT_FT_CONN_TYPE)));

		Nan::Set(features_obj, Nan::New("relay").ToLocalChecked(),
			Nan::New<v8::Boolean>(!!(oinfo.features & TANOIO_OUT_FT_RELAY)));

		Nan::Set(features_obj, Nan::New("safe_state").ToLocalChecked(),
			Nan::New<v8::Boolean>(!!(oinfo.features & TANOIO_OUT_FT_SAFE_STATE)));

		Nan::Set(features_obj, Nan::New("active_state").ToLocalChecked(),
			Nan::New<v8::Boolean>(!!(oinfo.features & TANOIO_OUT_FT_ACTIVE_STATE)));

		Nan::Set(features_obj, Nan::New("mode_direct").ToLocalChecked(),
			Nan::New<v8::Boolean>(!!(oinfo.features & TANOIO_OUT_FT_MODE_DIRECT)));

		Nan::Set(features_obj, Nan::New("mode_pwm").ToLocalChecked(),
			Nan::New<v8::Boolean>(!!(oinfo.features & TANOIO_OUT_FT_MODE_PWM)));

		Nan::Set(features_obj, Nan::New("mode_pulsegen").ToLocalChecked(),
			Nan::New<v8::Boolean>(!!(oinfo.features & TANOIO_OUT_FT_MODE_PULSEGEN)));

		Nan::Set(return_obj, Nan::New("features_u32").ToLocalChecked(),
			Nan::New<v8::Uint32>(oinfo.features));

		Nan::Set(return_obj, Nan::New("features").ToLocalChecked(), features_obj);

		OBJECT_SET_U32(return_obj, oinfo, type);

		info.GetReturnValue().Set(scope.Escape(return_obj));
	}

	static NAN_METHOD(out_get_params)
	{
		Nan::EscapableHandleScope scope;

		// Return object
		Local<Object> return_obj = Nan::New<v8::Object>();

		tanoio_handle_t handle = Nan::To<int>(info[0]).FromJust();
		unsigned int id = Nan::To<unsigned int>(info[1]).FromJust();

		tanoio_status_t status;
		struct tanoio_out_info oinfo;
		struct tanoio_out_params oparams;

		status = tanoio_out_get_info(handle, id, &oinfo);
		if (status != TANOIO_E_OK)
			throw_tanoio_error(status, NULL);

		status = tanoio_out_get_params(handle, id, &oparams);
		if (status != TANOIO_E_OK)
			throw_tanoio_error(status, NULL);

		OBJECT_SET_U32(return_obj, oinfo, id);

		OBJECT_SET_U32(return_obj, oparams, mode);
		OBJECT_SET_BOOL(return_obj, oparams, state);

		if (oinfo.features & TANOIO_OUT_FT_SAFE_STATE)
			OBJECT_SET_BOOL(return_obj, oparams, safe_state);

		if ((oparams.mode == TANOIO_OUT_MODE_PWM) ||
		    (oparams.mode == TANOIO_OUT_MODE_PULSEGEN))
		{
			OBJECT_SET_U32(return_obj, oparams, pwm_period_us);
			OBJECT_SET_U32(return_obj, oparams, pwm_duty_us);
		}

		if (oparams.mode == TANOIO_OUT_MODE_PULSEGEN)
		{
			OBJECT_SET_U32(return_obj, oparams, pulsegen_count);
			OBJECT_SET_U32(return_obj, oparams, pulsegen_preset_count);
		}

		if (oinfo.features & TANOIO_OUT_FT_CONN_TYPE)
			OBJECT_SET_U32(return_obj, oparams, conn_type);

		if (oinfo.features & TANOIO_OUT_FT_ACCIDENTS)
		{
			Nan::Set(return_obj, Nan::New("accident").ToLocalChecked(),
				Nan::New<v8::Boolean>(!!(oparams.flags & TANOIO_OUT_FL_ACCIDENT)));

			Nan::Set(return_obj, Nan::New("accident_watch").ToLocalChecked(),
				Nan::New<v8::Boolean>(!!(oparams.flags & TANOIO_OUT_FL_ACCIDENT_WATCH)));
		}

		if (oinfo.features & TANOIO_OUT_FT_ACCIDENTS_COUNT)
		{
			Nan::Set(return_obj, Nan::New("accidents_count").ToLocalChecked(),
				Nan::New<v8::Uint32>((unsigned int)oparams.accidents_count));
		}

		if (oinfo.features & TANOIO_OUT_FT_ACCIDENT_TYPE)
			OBJECT_SET_U32(return_obj, oparams, accident_type);

		info.GetReturnValue().Set(scope.Escape(return_obj));
	}

	static NAN_METHOD(out_set_states_mask)
	{
		Nan::EscapableHandleScope scope;

		tanoio_handle_t handle = Nan::To<int>(info[0]).FromJust();
		tanoio_mask_t set = Nan::To<unsigned int>(info[1]).FromJust();
		tanoio_mask_t mask = Nan::To<unsigned int>(info[2]).FromJust();

		tanoio_status_t status;

		status = tanoio_out_set_states_mask(handle, set, mask);
		info.GetReturnValue().Set(Nan::New<v8::Number>(status));
	}

	static NAN_METHOD(out_get_states_mask)
	{
		Nan::EscapableHandleScope scope;

		tanoio_handle_t handle = Nan::To<int>(info[0]).FromJust();
		tanoio_mask_t value;

		tanoio_status_t status;

		status = tanoio_out_get_states_mask(handle, &value);
		if (status != TANOIO_E_OK)
			throw_tanoio_error(status, NULL);

		info.GetReturnValue().Set(Nan::New<v8::Number>(value));
	}

	static NAN_METHOD(out_set_safe_states_mask)
	{
		Nan::EscapableHandleScope scope;

		tanoio_handle_t handle = Nan::To<int>(info[0]).FromJust();
		tanoio_mask_t set = (tanoio_mask_t)Nan::To<unsigned int>(info[1]).FromJust();
		tanoio_mask_t mask = (tanoio_mask_t)Nan::To<unsigned int>(info[2]).FromJust();

		tanoio_status_t status;

		status = tanoio_out_set_safe_states_mask(handle, set, mask);
		info.GetReturnValue().Set(Nan::New<v8::Number>(status));
	}

	static NAN_METHOD(out_get_safe_states_mask)
	{
		Nan::EscapableHandleScope scope;

		tanoio_handle_t handle = Nan::To<int>(info[0]).FromJust();
		tanoio_mask_t value;

		tanoio_status_t status;

		status = tanoio_out_get_safe_states_mask(handle, &value);
		if (status != TANOIO_E_OK)
			throw_tanoio_error(status, NULL);

		info.GetReturnValue().Set(Nan::New<v8::Number>(value));
	}

	static NAN_METHOD(out_set_mode)
	{
		TANOIO_CALL_FUNC_DEF_SET(
			tanoio_out_set_mode,
			tanoio_out_mode_t, v8::Number);
	}

	static NAN_METHOD(out_get_mode)
	{
		TANOIO_CALL_FUNC_DEF_GET(
			tanoio_out_get_mode,
			tanoio_out_mode_t, v8::Number);
	}

	static NAN_METHOD(out_set_state)
	{
		TANOIO_CALL_FUNC_DEF_SET(
			tanoio_out_set_state,
			tanoio_state_t, v8::Boolean);
	}

	static NAN_METHOD(out_get_state)
	{
		TANOIO_CALL_FUNC_DEF_GET(
			tanoio_out_get_state,
			tanoio_state_t, v8::Boolean);
	}

	static NAN_METHOD(out_set_safe_state)
	{
		TANOIO_CALL_FUNC_DEF_SET(
			tanoio_out_set_safe_state,
			tanoio_state_t, v8::Boolean);
	}

	static NAN_METHOD(out_get_safe_state)
	{
		TANOIO_CALL_FUNC_DEF_GET(
			tanoio_out_get_safe_state,
			tanoio_state_t, v8::Boolean);
	}

	static NAN_METHOD(out_set_mode_direct)
	{
		Nan::EscapableHandleScope scope;

		tanoio_handle_t handle = Nan::To<int>(info[0]).FromJust();
		unsigned int id = Nan::To<unsigned int>(info[1]).FromJust();
		tanoio_state_t state = (tanoio_state_t)Nan::To<unsigned int>(info[2]).FromJust();

		tanoio_status_t status;

		status = tanoio_out_set_mode_direct(handle, id, state);
		info.GetReturnValue().Set(Nan::New<v8::Number>(status));
	}

	static NAN_METHOD(out_set_mode_pwm)
	{
		Nan::EscapableHandleScope scope;

		tanoio_handle_t handle = Nan::To<int>(info[0]).FromJust();
		unsigned int id = Nan::To<unsigned int>(info[1]).FromJust();
		unsigned int period_us = Nan::To<unsigned int>(info[2]).FromJust();
		unsigned int duty_us = Nan::To<unsigned int>(info[3]).FromJust();

		tanoio_status_t status;

		status = tanoio_out_set_mode_pwm(handle, id, period_us, duty_us);
		info.GetReturnValue().Set(Nan::New<v8::Number>(status));
	}

	static NAN_METHOD(out_set_pwm_duty)
	{
		TANOIO_CALL_FUNC_DEF_SET(
			tanoio_out_set_pwm_duty,
			unsigned int, v8::Number);
	}

	static NAN_METHOD(out_get_pwm_duty)
	{
		TANOIO_CALL_FUNC_DEF_GET(
			tanoio_out_get_pwm_duty,
			unsigned int, v8::Number);
	}

	static NAN_METHOD(out_set_pwm_period)
	{
		TANOIO_CALL_FUNC_DEF_SET(
			tanoio_out_set_pwm_period,
			unsigned int, v8::Number);
	}

	static NAN_METHOD(out_get_pwm_period)
	{
		TANOIO_CALL_FUNC_DEF_GET(
			tanoio_out_get_pwm_period,
			unsigned int, v8::Number);
	}

	static NAN_METHOD(out_set_mode_pulsegen)
	{
		Nan::EscapableHandleScope scope;

		tanoio_handle_t handle = Nan::To<int>(info[0]).FromJust();
		unsigned int id = Nan::To<unsigned int>(info[1]).FromJust();
		unsigned int count = Nan::To<unsigned int>(info[2]).FromJust();
		unsigned int period_us = Nan::To<unsigned int>(info[3]).FromJust();
		unsigned int duty_us = Nan::To<unsigned int>(info[4]).FromJust();

		tanoio_status_t status;

		status = tanoio_out_set_mode_pulsegen(handle, id, count, period_us, duty_us);
		info.GetReturnValue().Set(Nan::New<v8::Number>(status));
	}

	static NAN_METHOD(out_set_pulsegen_count)
	{
		TANOIO_CALL_FUNC_DEF_SET(
			tanoio_out_set_pulsegen_count,
			unsigned int, v8::Number);
	}

	static NAN_METHOD(out_get_pulsegen_count)
	{
		TANOIO_CALL_FUNC_DEF_GET(
			tanoio_out_get_pulsegen_count,
			unsigned int, v8::Number);
	}

	static NAN_METHOD(out_get_pulsegen_preset_count)
	{
		TANOIO_CALL_FUNC_DEF_GET(
			tanoio_out_get_pulsegen_preset_count,
			unsigned int, v8::Number);
	}

	static NAN_METHOD(out_set_conn_type)
	{
		TANOIO_CALL_FUNC_DEF_SET(
			tanoio_out_set_conn_type,
			tanoio_out_conn_type_t, v8::Number);
	}

	static NAN_METHOD(out_get_conn_type)
	{
		TANOIO_CALL_FUNC_DEF_GET(
			tanoio_out_get_conn_type,
			tanoio_out_conn_type_t, v8::Number);
	}

	static NAN_METHOD(out_set_accident_watch)
	{
		TANOIO_CALL_FUNC_DEF_SET(
			tanoio_out_set_accident_watch,
			tanoio_bool_t, v8::Boolean);
	}

	static NAN_METHOD(out_get_accident_watch)
	{
		TANOIO_CALL_FUNC_DEF_GET(
			tanoio_out_get_accident_watch,
			tanoio_bool_t, v8::Boolean);
	}

	static NAN_METHOD(out_get_accident)
	{
		TANOIO_CALL_FUNC_DEF_GET(
			tanoio_out_get_accident,
			tanoio_bool_t, v8::Boolean);
	}

	static NAN_METHOD(out_get_accident_type)
	{
		TANOIO_CALL_FUNC_DEF_GET(
			tanoio_out_get_accident_type,
			tanoio_out_accident_type_t, v8::Boolean);
	}

	static NAN_METHOD(out_get_accidents_count)
	{
		TANOIO_CALL_FUNC_DEF_GET(
			tanoio_out_get_accidents_count,
			unsigned long long, v8::Number);
	}

	static NAN_METHOD(out_clear_accident)
	{
		TANOIO_CALL_FUNC_DEF(tanoio_out_clear_accident);
	}

	static NAN_METHOD(out_set_active_state)
	{
		TANOIO_CALL_FUNC_DEF_SET(
			tanoio_out_set_active_state,
			tanoio_active_state_t, v8::Boolean);
	}

	static NAN_METHOD(out_get_active_state)
	{
		TANOIO_CALL_FUNC_DEF_GET(
			tanoio_out_get_active_state,
			tanoio_active_state_t, v8::Boolean);
	}

	/*
	 *  _____       _ _
	 * |_   _|     (_) |
	 *   | |  _ __  _| |_
	 *   | | | '_ \| | __|
	 *  _| |_| | | | | |_
	 * |_____|_| |_|_|\__|
	 *
	 */
	static void Initialize(Handle<Object> target)
	{
		Nan::HandleScope scope;

		// Status codes
		NODE_DEFINE_CONSTANT(target, TANOIO_E_OK);
		NODE_DEFINE_CONSTANT(target, TANOIO_E_INVAL);
		NODE_DEFINE_CONSTANT(target, TANOIO_E_INVAL_ID);
		NODE_DEFINE_CONSTANT(target, TANOIO_E_INVAL_MODE);
		NODE_DEFINE_CONSTANT(target, TANOIO_E_NOMEM);
		NODE_DEFINE_CONSTANT(target, TANOIO_E_NODATA);
		NODE_DEFINE_CONSTANT(target, TANOIO_E_UNSUPPORTED);
		NODE_DEFINE_CONSTANT(target, TANOIO_E_RETRY);
		NODE_DEFINE_CONSTANT(target, TANOIO_E_TIMEOUT);
		NODE_DEFINE_CONSTANT(target, TANOIO_E_SYS);
		NODE_DEFINE_CONSTANT(target, TANOIO_E_IO_ERROR);
		NODE_DEFINE_CONSTANT(target, TANOIO_E_IPC_SEND);
		NODE_DEFINE_CONSTANT(target, TANOIO_E_IPC_RECV);
		NODE_DEFINE_CONSTANT(target, TANOIO_E_IPC_NOFUNC);
		NODE_DEFINE_CONSTANT(target, TANOIO_E_IPC_INVAL_REQUEST);
		NODE_DEFINE_CONSTANT(target, TANOIO_E_IPC_INVAL_REPLY);
		NODE_DEFINE_CONSTANT(target, TANOIO_E_IPC_CONNECTION);
		NODE_DEFINE_CONSTANT(target, TANOIO_E_IPC_SOCKET);

		// Boolean
		NODE_DEFINE_CONSTANT(target, TANOIO_TRUE);
		NODE_DEFINE_CONSTANT(target, TANOIO_FALSE);

		// States
		NODE_DEFINE_CONSTANT(target, TANOIO_STATE_DISABLED);
		NODE_DEFINE_CONSTANT(target, TANOIO_STATE_ENABLED);

		// Active states
		NODE_DEFINE_CONSTANT(target, TANOIO_ACTIVE_STATE_LOW);
		NODE_DEFINE_CONSTANT(target, TANOIO_ACTIVE_STATE_HIGH);

		// Edges
		NODE_DEFINE_CONSTANT(target, TANOIO_EDGE_NONE);
		NODE_DEFINE_CONSTANT(target, TANOIO_EDGE_RISING);
		NODE_DEFINE_CONSTANT(target, TANOIO_EDGE_FALLING);
		NODE_DEFINE_CONSTANT(target, TANOIO_EDGE_BOTH);

		// Input types
		NODE_DEFINE_CONSTANT(target, TANOIO_IN_TYPE_DI);
		NODE_DEFINE_CONSTANT(target, TANOIO_IN_TYPE_FDI);
		NODE_DEFINE_CONSTANT(target, TANOIO_IN_TYPE_GPIO);

		// Input modes
		NODE_DEFINE_CONSTANT(target, TANOIO_IN_MODE_UNKNOWN);
		NODE_DEFINE_CONSTANT(target, TANOIO_IN_MODE_COUNTER);
		NODE_DEFINE_CONSTANT(target, TANOIO_IN_MODE_AB_ENCODER);
		NODE_DEFINE_CONSTANT(target, TANOIO_IN_MODE_ABZ_ENCODER);
		NODE_DEFINE_CONSTANT(target, TANOIO_IN_MODE_DUTY_RATIO);

		// Input features
		NODE_DEFINE_CONSTANT(target, TANOIO_IN_FT_DEBOUNCE);
		NODE_DEFINE_CONSTANT(target, TANOIO_IN_FT_RESET);
		NODE_DEFINE_CONSTANT(target, TANOIO_IN_FT_FILTER);
		NODE_DEFINE_CONSTANT(target, TANOIO_IN_FT_MODE_COUNTER);
		NODE_DEFINE_CONSTANT(target, TANOIO_IN_FT_MODE_AB_ENCODER);
		NODE_DEFINE_CONSTANT(target, TANOIO_IN_FT_MODE_ABZ_ENCODER);
		NODE_DEFINE_CONSTANT(target, TANOIO_IN_FT_MODE_DUTY_RATIO);

		// Input flags
		NODE_DEFINE_CONSTANT(target, TANOIO_IN_FL_DEBOUNCE);

		// Output types
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_TYPE_DO);
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_TYPE_FDO);
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_TYPE_LED);
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_TYPE_GPIO);

		// Output modes
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_MODE_UNKNOWN);
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_MODE_DIRECT);
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_MODE_PWM);
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_MODE_PULSEGEN);

		// Output features
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_FT_ACCIDENTS);
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_FT_ACCIDENTS_COUNT);
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_FT_ACCIDENT_TYPE);
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_FT_CONN_TYPE);
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_FT_RELAY);
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_FT_SAFE_STATE);
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_FT_MODE_DIRECT);
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_FT_MODE_PWM);
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_FT_MODE_PULSEGEN);

		// Output flags
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_FL_ACCIDENT);
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_FL_ACCIDENT_WATCH);

		// Output connection types
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_CONN_TYPE_UNKNOWN);
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_CONN_TYPE_HIGH_SIDE);
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_CONN_TYPE_PUSH_PULL);

		// Output accident types
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_ACCIDENT_TYPE_NONE);
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_ACCIDENT_TYPE_UVLO);
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_ACCIDENT_TYPE_OVERHEAT);
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_ACCIDENT_TYPE_SHORT_CIRCUIT);
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_ACCIDENT_TYPE_OPEN_LOAD);
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_ACCIDENT_TYPE_BAD);

		// Module features
		NODE_DEFINE_CONSTANT(target, TANOIO_MODULE_FT_IN_STATES_MASK);
		NODE_DEFINE_CONSTANT(target, TANOIO_MODULE_FT_OUT_STATES_MASK);
		NODE_DEFINE_CONSTANT(target, TANOIO_MODULE_FT_OUT_SAFE_TIMEOUT);
		NODE_DEFINE_CONSTANT(target, TANOIO_MODULE_FT_OUT_SAFE_STATES_MASK);
		NODE_DEFINE_CONSTANT(target, TANOIO_MODULE_FT_MANUAL_SYNC);

		// Module flags
		NODE_DEFINE_CONSTANT(target, TANOIO_MODULE_FL_MANUAL_SYNC);

		// Events
		NODE_DEFINE_CONSTANT(target, TANOIO_EVENT_IN_STATE_CHANGED);
		NODE_DEFINE_CONSTANT(target, TANOIO_EVENT_OUT_ACCIDENT);

		// Other
		NODE_DEFINE_CONSTANT(target, TANOIO_MASK_ALL);
		NODE_DEFINE_CONSTANT(target, TANOIO_VERSION);
		NODE_DEFINE_CONSTANT(target, TANOIO_VERSION_MAJOR);
		NODE_DEFINE_CONSTANT(target, TANOIO_VERSION_MINOR);
		NODE_DEFINE_CONSTANT(target, TANOIO_VERSION_FIX);
		NODE_DEFINE_CONSTANT(target, TANOIO_IN_NAME_MAX_SIZE);
		NODE_DEFINE_CONSTANT(target, TANOIO_IN_DESC_MAX_SIZE);
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_RELAY_MIN_PERIOD_US);
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_RELAY_MIN_HIGH_PULSE_US);
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_RELAY_MIN_LOW_PULSE_US);
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_NAME_MAX_SIZE);
		NODE_DEFINE_CONSTANT(target, TANOIO_OUT_DESC_MAX_SIZE);
		NODE_DEFINE_CONSTANT(target, TANOIO_MODULE_HARDWARE_ID_MAX_SIZE);
		NODE_DEFINE_CONSTANT(target, TANOIO_MODULE_NAME_MAX_SIZE);
		NODE_DEFINE_CONSTANT(target, TANOIO_MODULE_DESCRIPTION_MAX_SIZE);
		NODE_DEFINE_CONSTANT(target, TANOIO_MAX_INPUTS);
		NODE_DEFINE_CONSTANT(target, TANOIO_MAX_OUTPUTS);

		target->Set(Nan::New("TANOIO_NN_DEFAULT_URL_IPC").ToLocalChecked(),
			Nan::New<String>(TANOIO_NN_DEFAULT_URL_IPC).ToLocalChecked());

		target->Set(Nan::New("TANOIO_NN_DEFAULT_URL_EVENTS").ToLocalChecked(),
			Nan::New<String>(TANOIO_NN_DEFAULT_URL_EVENTS).ToLocalChecked());

		target->Set(Nan::New("TANOIO_ULOG_IDENT").ToLocalChecked(),
			Nan::New<String>(TANOIO_ULOG_IDENT).ToLocalChecked());

		target->Set(Nan::New("TANOIO_UCI_PACKAGE").ToLocalChecked(),
			Nan::New<String>(TANOIO_UCI_PACKAGE).ToLocalChecked());

		// Methods
		#define EXPORT_METHOD(method) \
			Nan::SetMethod(target, # method, method)

		// Control methods
		EXPORT_METHOD(init);
		EXPORT_METHOD(destroy);
		EXPORT_METHOD(get_info);

		// Modules methods
		EXPORT_METHOD(module_get_info);
		EXPORT_METHOD(module_get_params);
		EXPORT_METHOD(module_out_set_safe_timeout);
		EXPORT_METHOD(module_out_get_safe_timeout);
		EXPORT_METHOD(module_get_manual_sync);
		EXPORT_METHOD(module_set_manual_sync);
		EXPORT_METHOD(module_sync);

		// Input methods
		EXPORT_METHOD(in_get_info);
		EXPORT_METHOD(in_get_params);
		EXPORT_METHOD(in_get_states_mask);
		EXPORT_METHOD(in_get_mode);
		EXPORT_METHOD(in_set_mode);
		EXPORT_METHOD(in_get_counter);
		EXPORT_METHOD(in_reset);
		EXPORT_METHOD(in_get_state);
		EXPORT_METHOD(in_get_debounce);
		EXPORT_METHOD(in_set_debounce);
		EXPORT_METHOD(in_set_filter_period);
		EXPORT_METHOD(in_get_filter_period);
		EXPORT_METHOD(in_set_filter_length);
		EXPORT_METHOD(in_get_filter_length);
		EXPORT_METHOD(in_get_measured_period);
		EXPORT_METHOD(in_get_measured_duty);
		EXPORT_METHOD(in_get_ab_encoder);
		EXPORT_METHOD(in_get_abz_encoder);
		EXPORT_METHOD(in_set_active_state);
		EXPORT_METHOD(in_get_active_state);
		EXPORT_METHOD(in_set_edge);
		EXPORT_METHOD(in_get_edge);

		// Output methods
		EXPORT_METHOD(out_get_info);
		EXPORT_METHOD(out_get_params);
		EXPORT_METHOD(out_set_states_mask);
		EXPORT_METHOD(out_get_states_mask);
		EXPORT_METHOD(out_set_safe_states_mask);
		EXPORT_METHOD(out_get_safe_states_mask);
		EXPORT_METHOD(out_set_mode);
		EXPORT_METHOD(out_get_mode);
		EXPORT_METHOD(out_set_state);
		EXPORT_METHOD(out_get_state);
		EXPORT_METHOD(out_set_safe_state);
		EXPORT_METHOD(out_get_safe_state);
		EXPORT_METHOD(out_set_mode_direct);
		EXPORT_METHOD(out_set_mode_pwm);
		EXPORT_METHOD(out_set_pwm_duty);
		EXPORT_METHOD(out_get_pwm_duty);
		EXPORT_METHOD(out_set_pwm_period);
		EXPORT_METHOD(out_get_pwm_period);
		EXPORT_METHOD(out_set_mode_pulsegen);
		EXPORT_METHOD(out_set_pulsegen_count);
		EXPORT_METHOD(out_get_pulsegen_count);
		EXPORT_METHOD(out_get_pulsegen_preset_count);
		EXPORT_METHOD(out_set_conn_type);
		EXPORT_METHOD(out_get_conn_type);
		EXPORT_METHOD(out_set_accident_watch);
		EXPORT_METHOD(out_get_accident_watch);
		EXPORT_METHOD(out_get_accident);
		EXPORT_METHOD(out_get_accident_type);
		EXPORT_METHOD(out_get_accidents_count);
		EXPORT_METHOD(out_clear_accident);
		EXPORT_METHOD(out_set_active_state);
		EXPORT_METHOD(out_get_active_state);
	}
};

extern "C"
{
	static void init (Handle<Object> target)
	{
		TanoIO::Initialize(target);
	}

	NODE_MODULE(tanoio, init)
}
