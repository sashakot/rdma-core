/*
 * Copyright (c) 2004-2009 Voltaire Inc.  All rights reserved.
 *
 * This software is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under the terms of the GNU
 * General Public License (GPL) Version 2, available from the file
 * COPYING in the main directory of this source tree, or the
 * OpenIB.org BSD license below:
 *
 *     Redistribution and use in source and binary forms, with or
 *     without modification, are permitted provided that the following
 *     conditions are met:
 *
 *      - Redistributions of source code must retain the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer.
 *
 *      - Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#define _GNU_SOURCE

#include <inttypes.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <endian.h>

#include <infiniband/mad.h>
#include <infiniband/umad.h>

#include <sys/time.h>


#include <ibdiag_common.h>

#define MAX_TARGET_QUEUE_DEPTH 128
#define MAX_SOURCE_QUEUE_DEPTH 1024

enum mngt_methods {
	mngt_method_get = 1,
	mngt_method_set = 2
};

float timedifference_msec(struct timeval t0, struct timeval t1);
int timedifference_usec(struct timeval t0, struct timeval t1);

static int drmad_tid = 0x123;

typedef struct {
	char path[64];
	int hop_cnt;
} DRPath;

struct mad_target {
	uint32_t lid;
	DRPath * path;
	int on_wire_mads;
	int send_mads;
	int timeouts;	// number of timeout responces from driver
	int errors;
	int ok_mads;	// number of ok responces from device
	int min_latency_us;
	int max_latency_us;
	int avrg_latency_us;
	uint64_t total_time_us; // total time of all mads on wire
	uint8_t data[64]; // data for set operation
};

struct mad_operation {
	be64_t tid; // Network order, valid high 32 bit
	struct mad_target *target;
	struct timeval start;
};

struct mad_buffer {
	void *umad;
	struct drsmp *smp;
	struct ib_user_mad *mad;
};

struct mad_worker {
	/*
	umad params
	*/
	int ibd_timeout;
	int ibd_retries;

	/*
	mad attributes
	*/
	int mgmt_class; // IB_SMI_DIRECT_CLASS , IB_SMI_CLASS
	int mngt_method; // 1 - Get, 2 - Set
	int smp_attr;
	int smp_mod;

	/*
	IB Device
	*/
	char ibd_ca[UMAD_CA_NAME_LEN];
	int ibd_ca_port;
	int mad_agent;
	int portid;

	/*
	target devices
	*/
	struct mad_target *targets;
	int n_targets;
	/*
	runtime
	*/
	int target_queue_depth;
	int source_queue_depth;
	void *umad;
	//struct mad_buffer mad;
	int last_device;
	int timeout_ms;
	struct timeval start;
	
	/*
	queue
	*/
	struct mad_operation *mads_on_wire;
};

int init_mad_worker(struct mad_worker *w);
int init_ib_device(struct mad_worker *w, const char *ibd_ca, int ibd_ca_port);
void finalize_mad_worker(struct mad_worker *w);
void check_worker(const struct mad_worker *w);
int process_mads(struct mad_worker *w);
void set_lid_routet_targets(struct mad_worker *w, uint32_t *lids, int n);
int send_mads(struct mad_worker *w);
void report_worker_params(struct mad_worker *w, FILE *f);
void print_statistics(struct mad_worker *w, FILE *f);
int fetch_attribute(struct mad_worker *w);

struct drsmp {
	uint8_t base_version;
	uint8_t mgmt_class;
	uint8_t class_version;
	uint8_t method;
	__be16 status;
	uint8_t hop_ptr;
	uint8_t hop_cnt;
	__be64 tid;
	__be16 attr_id;
	uint16_t resv;
	__be32 attr_mod;
	__be64 mkey;
	__be16 dr_slid;
	__be16 dr_dlid;
	uint8_t reserved[28];
	uint8_t data[64];
	uint8_t initial_path[64];
	uint8_t return_path[64];
};

static void drsmp_get_init(void *umad, DRPath * path, int attr, int mod, int mngt_method, uint8_t data[64])
{
	struct drsmp *smp = (struct drsmp *)(umad_get_mad(umad));

	memset(smp, 0, sizeof(*smp));

	smp->base_version = 1;
	smp->mgmt_class = IB_SMI_DIRECT_CLASS;
	smp->class_version = 1;

	smp->method = mngt_method;
	smp->attr_id = htons(attr);
	smp->attr_mod = htonl(mod);
	smp->tid = htobe64(drmad_tid);
	drmad_tid++;
	smp->dr_slid = htobe16(0xffff);
	smp->dr_dlid = htobe16(0xffff);

	umad_set_addr(umad, 0xffff, 0, 0, 0);

	if (path)
		memcpy(smp->initial_path, path->path, path->hop_cnt + 1);

	smp->hop_cnt = (uint8_t) path->hop_cnt;

	if (mngt_method == mngt_method_set && data)
		memcpy(smp->data, data, 64);
}

static void smp_get_init(void *umad, int lid, int attr, int mod, int mngt_method, uint8_t data[64])
{
	struct drsmp *smp = (struct drsmp *)(umad_get_mad(umad));

	memset(smp, 0, sizeof(*smp)-8);

	smp->base_version = 1;
	smp->mgmt_class = IB_SMI_CLASS;
	smp->class_version = 1;

	smp->method = mngt_method;
	smp->attr_id = htons(attr);
	smp->attr_mod = htonl(mod);
	smp->tid = htobe64(drmad_tid);
	drmad_tid++;

	if (mngt_method == mngt_method_set && data)
		memcpy(smp->data, data, 64);

	umad_set_addr(umad, lid, 0, 0, 0);
}

static int str2DRPath(char *str, DRPath * path)
{
	char *s;

	path->hop_cnt = -1;

	DEBUG("DR str: %s", str);
	while (str && *str) {
		if ((s = strchr(str, ',')))
			*s = 0;
		path->path[++path->hop_cnt] = (char)atoi(str);
		if (!s)
			break;
		str = s + 1;
	}

	return path->hop_cnt;
}

static int dump_char;

static int process_opt(void *context, int ch)
{
	struct mad_worker *w = (struct mad_worker *)context;

	assert(w != 0);

	switch (ch) {
	case 's':
		dump_char++;
		break;
	case 'D':
		w->mgmt_class = IB_SMI_DIRECT_CLASS;
		break;
	case 'L':
		w->mgmt_class = IB_SMI_CLASS;
		break;
	case 'm':
		w->mngt_method = (uint64_t) strtoull(optarg, NULL, 0);
		break;
	case 'N':
		w->source_queue_depth = w->target_queue_depth = (uint64_t) strtoull(optarg, NULL, 0);
		break;
	case 't':
		w->timeout_ms = (uint64_t) strtoull(optarg, NULL, 0);
		w->timeout_ms *= 1000;
		break;
	case 'r':
		w->ibd_retries = (uint64_t) strtoull(optarg, NULL, 0);
		break;
	case 'T':
		w->ibd_timeout = (uint64_t) strtoull(optarg, NULL, 0);
		break;
	default:
		return -1;
	}
	return 0;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"


int init_mad_worker(struct mad_worker *w)
{
	w->ibd_timeout = 200;
	w->ibd_retries = 3;
	w->mgmt_class = IB_SMI_CLASS;
	w->mngt_method = 1; // Get
	w->smp_attr = 0;
	w->smp_mod = 0;
	w->source_queue_depth = w->target_queue_depth = 1;


	w->last_device = 0;

	w->targets = NULL;
	w->n_targets = 0;

	w->ibd_ca[0] = 0;
	w->ibd_ca_port = 0;
	w->portid = -1;

	w->timeout_ms = 0;
	return 0;
}

int init_ib_device(struct mad_worker *w, const char *ca, int ca_port)
{
	if (ca)
		strncpy(w->ibd_ca, ibd_ca,UMAD_CA_NAME_LEN -1);
	w->ibd_ca_port = ibd_ca_port;

	if ((w->portid = umad_open_port(ibd_ca, ibd_ca_port)) < 0)
		IBPANIC("can't open UMAD port (%s:%d)", ibd_ca, ibd_ca_port);

	if ((w->mad_agent = umad_register(w->portid, w->mgmt_class, 1, 0, NULL)) < 0)
		IBPANIC("Couldn't register agent for SMPs");

	if ( !( w->umad = umad_alloc(1, umad_size() + IB_MAD_SIZE)))
		IBPANIC("can't alloc MAD");

	w->mads_on_wire = (struct mad_operation *)calloc(1, w->source_queue_depth * sizeof(w->mads_on_wire[0]));
	if (!w->mads_on_wire)
		IBPANIC("Can't allocate mad queue");
	return 0;
}

void set_lid_routet_targets(struct mad_worker *w, uint32_t *lids, int n)
{
	int i;

	w->targets = (struct mad_target *)calloc(1, n * sizeof(w->targets[0]));
	if(!w->targets)
		IBPANIC("can't allocate list of devices");

	w->n_targets = n;

	for (i = 0; i < n; ++i)
		w->targets[i].lid = lids[i];
}

int fetch_attribute(struct mad_worker *w)
{
	int i, rc, length, status;
	struct drsmp *smp;

	if(w->mngt_method != mngt_method_set)
		return -1;

	for (i = 0; i < w->n_targets; i++) {
		if (w->mgmt_class == IB_SMI_DIRECT_CLASS)
				drsmp_get_init(w->umad, w->targets[i].path, w->smp_attr, w->smp_mod, mngt_method_get, NULL); // TODO: Fix
			else
				smp_get_init(w->umad, w->targets[i].lid, w->smp_attr, w->smp_mod, mngt_method_get, NULL); // Get attribute

			rc = umad_send(w->portid, w->mad_agent, w->umad, IB_MAD_SIZE, 1000, 3); // hardcoded timeout and retries. This send is only preprocessing
			if (rc)
				IBPANIC("send failed rc : %d", rc);

			length = IB_MAD_SIZE;
			rc = umad_recv(w->portid, w->umad, &length, -1);
			if (rc != w->mad_agent)
				IBPANIC("recv error: %d %m", rc);

			status = umad_status(w->umad);
			if (status == ETIMEDOUT)
				IBPANIC("mad timeout");

			smp = (struct drsmp *)(umad_get_mad(w->umad));

			memcpy(w->targets[i].data, smp->data, 64);
	}

	return 0;
}

int send_mads(struct mad_worker *w)
{
	int i, j, rc;
	struct mad_target *target;
	int idx;
	struct drsmp *smp = (struct drsmp *)(umad_get_mad(w->umad));

	for (i = 0; i < w->source_queue_depth; ++i) {
		if (!w->mads_on_wire[i].tid) {

			for(j = 0; j < w->n_targets; ++j) {
				idx = (w->last_device + 1 +j) % w->n_targets;
				target = &w->targets[idx];
				if (target->on_wire_mads < w->target_queue_depth)
					break;
			}

			if (j == w->n_targets)
				continue;

			if (w->mgmt_class == IB_SMI_DIRECT_CLASS)
				drsmp_get_init(w->umad, w->targets[idx].path, w->smp_attr, w->smp_mod, w->mngt_method, w->targets[idx].data); // TODO: Fix
			else
				smp_get_init(w->umad, w->targets[idx].lid, w->smp_attr, w->smp_mod, w->mngt_method, w->targets[idx].data);

			rc = umad_send(w->portid, w->mad_agent, w->umad, IB_MAD_SIZE, w->ibd_timeout, w->ibd_retries);
			if (rc)
				IBPANIC("send failed rc : %d", rc);

			gettimeofday(&w->mads_on_wire[i].start, NULL);
			w->mads_on_wire[i].tid = smp->tid;
			w->mads_on_wire[i].target = target;
			w->last_device = idx;
			target->on_wire_mads++;
			target->send_mads++;
		}
	}
	return 0;
}

inline float timedifference_msec(struct timeval t0, struct timeval t1)
{
    return (t1.tv_sec - t0.tv_sec) * 1000.0f + (t1.tv_usec - t0.tv_usec) / 1000.0f;
}

inline int timedifference_usec(struct timeval t0, struct timeval t1)
{
	return timedifference_msec(t0, t1) * 1000;
}

int process_mads(struct mad_worker *w)
{
	float time_left_ms;
	struct timeval current;
	int i, rc ,status, length;
	be64_t tid;
	int latency;
	struct mad_target *target;
	struct drsmp *smp = (struct drsmp *)(umad_get_mad(w->umad));

	if(w->mngt_method == mngt_method_set) {
		rc = fetch_attribute(w);
		if (rc)
			IBPANIC("fetch attribute value is failed");
	}

	gettimeofday(&w->start, NULL);

	while (1) {
		gettimeofday(&current, NULL);

		time_left_ms = w->timeout_ms - timedifference_msec(w->start, current);
		if (time_left_ms <= 0)
			return 0;

		send_mads(w);

		rc = umad_poll(w->portid, (int)time_left_ms);
		if (rc == -ETIMEDOUT)
			return 0;
		else if (rc)
			IBPANIC("umad_poll failed: %d %m", rc);

		length = IB_MAD_SIZE;
		rc = umad_recv(w->portid, w->umad, &length, -1);
		if (rc != w->mad_agent) {
			IBPANIC("recv error: %d %m", rc);
		}

		gettimeofday(&current, NULL);
		status = umad_status(w->umad);

		tid = smp->tid >> 32;

		for (i = 0; i < w->source_queue_depth; ++i) {
			be64_t t = w->mads_on_wire[i].tid >> 32;
			if (t == tid)
				break;
		}

		if (i < w->source_queue_depth) {
			target = w->mads_on_wire[i].target;
			target->on_wire_mads--;
			if (status == ETIMEDOUT)
				target->timeouts++;
			else if (status)
				target->errors++;
			else
				target->ok_mads++;

			latency = timedifference_usec(w->mads_on_wire[i].start, current);

			if (latency > target->max_latency_us)
				target->max_latency_us = latency;
			if (latency < target->min_latency_us || !target->min_latency_us)
				target->min_latency_us = latency;

			target->total_time_us += latency;
			target->avrg_latency_us = target->total_time_us / (target->timeouts + target->errors + target->ok_mads);

			w->mads_on_wire[i].tid = 0;
			w->mads_on_wire[i].target = NULL;
		} else {
			IBWARN("tid %ld is not found", be64toh(tid));
		}
	}
}

void finalize_mad_worker(struct mad_worker *w)
{
	umad_free(w->umad);

	umad_unregister(w->portid, w->mad_agent);
	umad_close_port(w->portid);

	free(w->mads_on_wire);
	free(w->targets);
}

void report_worker_params(struct mad_worker *w, FILE *f)
{
	fprintf(f, "device: %s port %d\n", w->ibd_ca, w->ibd_ca_port);
	fprintf(f, "umad timeout: %d  retries: %d\n ", w->ibd_timeout, w->ibd_retries);
	fprintf(f, "mngt class %s (%d)\n ", w->mgmt_class ==  IB_SMI_CLASS? "IB_SMI_CLASS" : "IB_SMI_DIRECT_CLASS", w->mgmt_class);
	fprintf(f, "mngt method %s (%d)\n ", w->mngt_method == 1 ? "GET" : "SET", w->mngt_method);
	fprintf(f, "source queue depth: %d , target queue depth: %d\n", w->source_queue_depth, w->target_queue_depth);
}

void print_statistics(struct mad_worker *w, FILE *f)
{
	int i;
	int send_mads = 0, ok_mads = 0, errors = 0, timeouts = 0 , total_mads = 0;
	int total_time = 0;
	int min_latency_us = 0, max_latency_us = 0, avrg_latency_us = 0;

	for (i = 0; i < w->n_targets; ++ i) {

		if (!w->targets[i].send_mads)
			continue;
			
		send_mads += w->targets[i].send_mads;
		ok_mads += w->targets[i].ok_mads;
		errors += w->targets[i].errors;
		timeouts += w->targets[i].timeouts;

		if (!min_latency_us || min_latency_us > w->targets[i].min_latency_us)
			min_latency_us = w->targets[i].min_latency_us;
		if (max_latency_us < w->targets[i].max_latency_us)
			max_latency_us = w->targets[i].max_latency_us;

		total_time += w->targets[i].total_time_us;
	}

	total_mads = ok_mads + errors + timeouts;
	avrg_latency_us = total_time / total_mads;

	fprintf(f, "Local device: %s , port: %d\n", strlen(w->ibd_ca) ? w->ibd_ca : "Default", w->ibd_ca_port);
	fprintf(f, "	send mads: %d , ok mads: %d , timeouts: %d , errors %d\n",  send_mads, ok_mads, timeouts, errors);
	fprintf(f, "	latency (us) min: %d , max:%d , average: %d\n",  min_latency_us, max_latency_us, avrg_latency_us);
	fprintf(f, "\n");

	for (i = 0; i < w->n_targets; ++ i) {
		fprintf(f, "lid: %d", w->targets[i].lid);
		fprintf(f, "	send mads: %d , ok mads: %d , timeouts: %d , errors %d\n",  w->targets[i].send_mads, w->targets[i].ok_mads, w->targets[i].timeouts, w->targets[i].errors);
		fprintf(f, "	latency (us) min: %d , max:%d , average: %d\n",  w->targets[i].min_latency_us, w->targets[i].max_latency_us, w->targets[i].avrg_latency_us);
		fprintf(f, "\n");
	}
}

void check_worker(const struct mad_worker *w)
{
	if (w->mngt_method != 1 && w->mngt_method != 2 )
		IBPANIC("wrong mngt method : %d", w->mngt_method);
	if (w->target_queue_depth > MAX_TARGET_QUEUE_DEPTH)
		IBPANIC("mad queue depth for destination device is too big: %d , max : %d", w->target_queue_depth, MAX_TARGET_QUEUE_DEPTH);
	if (w->source_queue_depth > MAX_SOURCE_QUEUE_DEPTH)
		IBPANIC("mad queue for local device is tool long : %d , max : %d", w->source_queue_depth, MAX_SOURCE_QUEUE_DEPTH);
	if (w->mgmt_class != IB_SMI_DIRECT_CLASS && w->mgmt_class != IB_SMI_CLASS)
		IBPANIC("wrong mngt method : %d", w->mgmt_class);
	if (w->source_queue_depth < w->target_queue_depth)
		IBWARN("local queue depth is lower than targetqueue depth %d < %d", w->source_queue_depth, w->target_queue_depth);
}

int main(int argc, char *argv[])
{
	int dlid = 0;
	DRPath path;
//	uint8_t *desc;
	struct mad_worker w;
	uint32_t lids[1024] = {};

	const struct ibdiag_opt opts[] = {
		{"string", 's', 0, NULL, ""},
		{"queue_depth", 'N', 1, "<queue_depth>", ""},
		{"run_time", 't', 1, "<time>", ""},
		{"mngt_method", 'm', 1, "<method>", ""},
		{"umad_retries", 'r', 1, "<retries>", ""},
		{"umad_timeout", 'T', 1, "<timeout ms>", ""},
		{}
	};
	char usage_args[] = "<dlid|dr_path> <attr> [mod]";
	const char *usage_examples[] = {
		" -- DR routed examples:",
		"-D 0,1,2,3,5 16	# NODE DESC",
		"-D 0,1,2 0x15 2	# PORT INFO, port 2",
		" -- LID routed examples:",
		"3 0x15 2	# PORT INFO, lid 3 port 2",
		"0xa0 0x11	# NODE INFO, lid 0xa0",
		NULL
	};

	init_mad_worker(&w);

	ibdiag_process_opts(argc, argv, &w, "GKs", opts, process_opt,
			    usage_args, usage_examples);

	check_worker(&w);

	argc -= optind;
	argv += optind;

	if (argc < 2)
		ibdiag_show_usage();

	if (w.mgmt_class == IB_SMI_DIRECT_CLASS &&
	    str2DRPath(strdupa(argv[0]), &path) < 0)
		IBPANIC("bad path str '%s'", argv[0]);

	if (w.mgmt_class == IB_SMI_CLASS)
		dlid = strtoul(argv[0], NULL, 0);

	w.smp_attr = strtoul(argv[1], NULL, 0);
	if (argc > 2)
		w.smp_mod = strtoul(argv[2], NULL, 0);

	if (umad_init() < 0)
		IBPANIC("can't init UMAD library");

	init_ib_device(&w, ibd_ca, ibd_ca_port);

	report_worker_params(&w, stdout);

	lids[0] = lids[1] = dlid;
	set_lid_routet_targets(&w, (uint32_t *)&lids, 2);

	//if (ibdebug > 1)
	//	xdump(stderr, "before send:\n", w.smp, 256);

	process_mads(&w);

	print_statistics(&w, stdout);
/*
	if (!dump_char) {
		xdump(stdout, NULL, w.mad.smp->data, 64);
		if (w.mad.smp->status)
			fprintf(stdout, "SMP status: 0x%x\n",
				ntohs(w.mad.smp->status));
		goto exit;
	}
*/
/*
	desc = w.mad.smp->data;
	for (i = 0; i < 64; ++i) {

		if (!desc[i])
			break;
		putchar(desc[i]);
	}
*/
	putchar('\n');
//	if (w.mad.smp->status)
//		fprintf(stdout, "SMP status: 0x%x\n", ntohs(w.mad.smp->status));

	finalize_mad_worker(&w);
	return 0;
}
