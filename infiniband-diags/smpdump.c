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

static int mad_agent;
static int drmad_tid = 0x123;
static int mngt_method = 0x01; // 1 - Get, 2 - Set
static uint64_t N = 1;
static int t_sec = 1;

typedef struct {
	char path[64];
	int hop_cnt;
} DRPath;

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

static void drsmp_get_init(void *umad, DRPath * path, int attr, int mod)
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
}

static void smp_get_init(void *umad, int lid, int attr, int mod)
{
	struct drsmp *smp = (struct drsmp *)(umad_get_mad(umad));

	memset(smp, 0, sizeof(*smp));

	smp->base_version = 1;
	smp->mgmt_class = IB_SMI_CLASS;
	smp->class_version = 1;

	smp->method = mngt_method;
	smp->attr_id = htons(attr);
	smp->attr_mod = htonl(mod);
	smp->tid = htobe64(drmad_tid);
	drmad_tid++;

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

#if 0
	if (path->path[0] != 0 ||
	    (path->hop_cnt > 0 && dev_port && path->path[1] != dev_port)) {
		DEBUG("hop 0 != 0 or hop 1 != dev_port");
		return -1;
	}
#endif

	return path->hop_cnt;
}

static int dump_char, mgmt_class = IB_SMI_CLASS;

static int process_opt(void *context, int ch)
{
	switch (ch) {
	case 's':
		dump_char++;
		break;
	case 'D':
		mgmt_class = IB_SMI_DIRECT_CLASS;
		break;
	case 'L':
		mgmt_class = IB_SMI_CLASS;
		break;
	case 'm':
		mngt_method = (uint64_t) strtoull(optarg, NULL, 0);
		break;
	case 'N':
		N = (uint64_t) strtoull(optarg, NULL, 0);
		break;
	case 't':
		t_sec = (uint64_t) strtoull(optarg, NULL, 0);
		break;
	default:
		return -1;
	}
	return 0;
}

struct smp_query_task {
	void *umad;
	struct drsmp *smp;
	struct timeval t1;
};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

int main(int argc, char *argv[])
{
	int dlid = 0;
	void *umad;
	struct drsmp *smp;
	int i, j, portid, mod = 0, attr;
	DRPath path;
	uint8_t *desc;
	int length;
	struct timeval t2;
	struct timeval start;
	struct timeval end;
    uint64_t elapsedTime;
	struct smp_query_task *smp_query_tasks;
	int do_poll = 0;
	uint64_t total_mads = 0;
	uint64_t timeout_mads = 0;
	uint64_t minTime = 0, maxTime = 0, avrgTime = 0, totalTime = 0;
	uint64_t totalSend = 0, totalRecv = 0;
	double runTime = 0;


	const struct ibdiag_opt opts[] = {
		{"string", 's', 0, NULL, ""},
		{"queue_depth", 'N', 1, "<queue_depth>", ""},
		{"run_time", 't', 1, "<time>", ""},
		{"mngt_method", 'm', 1, "<method>", ""},
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

	ibd_timeout = 1000;
	int ibd_retries = 0;

	ibdiag_process_opts(argc, argv, NULL, "GKs", opts, process_opt,
			    usage_args, usage_examples);

	argc -= optind;
	argv += optind;

	if (argc < 2)
		ibdiag_show_usage();

	if (mgmt_class == IB_SMI_DIRECT_CLASS &&
	    str2DRPath(strdupa(argv[0]), &path) < 0)
		IBPANIC("bad path str '%s'", argv[0]);

	if (mgmt_class == IB_SMI_CLASS)
		dlid = strtoul(argv[0], NULL, 0);

	attr = strtoul(argv[1], NULL, 0);
	if (argc > 2)
		mod = strtoul(argv[2], NULL, 0);

	if (mngt_method !=1 && mngt_method !=2)
		IBPANIC("wrong umad method %d", mngt_method);	

	printf("umad timeout %d retries %d\n", ibd_timeout, ibd_retries);
	printf("umad method %d\n",mngt_method);
	printf("umad queue %ld\n", N);
	printf("running time in sec %d\n", t_sec);

	if (umad_init() < 0)
		IBPANIC("can't init UMAD library");

	if ((portid = umad_open_port(ibd_ca, ibd_ca_port)) < 0)
		IBPANIC("can't open UMAD port (%s:%d)", ibd_ca, ibd_ca_port);

	if ((mad_agent = umad_register(portid, mgmt_class, 1, 0, NULL)) < 0)
		IBPANIC("Couldn't register agent for SMPs");

	if (!(umad = umad_alloc(1, umad_size() + IB_MAD_SIZE)))
		IBPANIC("can't alloc MAD");

	smp = umad_get_mad(umad);

	smp_query_tasks = (struct smp_query_task*)malloc(N * sizeof(smp_query_tasks[0]));
	if (!smp_query_tasks)
		IBPANIC("can't alloc list of tasks");
	memset(smp_query_tasks, 0, N * sizeof(smp_query_tasks[0]));
	for (i = 0; i < N; ++i) {
		smp_query_tasks[i].umad = umad_alloc(1, umad_size() + IB_MAD_SIZE);
		if (!smp_query_tasks[i].umad)
			IBPANIC("can't alloc MAD");

		smp_query_tasks[i].smp = umad_get_mad(smp_query_tasks[i].umad);

		if (mgmt_class == IB_SMI_DIRECT_CLASS)
			drsmp_get_init(smp_query_tasks[i].umad, &path, attr, mod);
		else
			smp_get_init(smp_query_tasks[i].umad, dlid, attr, mod);
		
		memset(smp_query_tasks[i].smp->data,0xff,64);
	}

	if (ibdebug > 1)
		xdump(stderr, "before send:\n", smp, 256);

	gettimeofday(&start, NULL);

	length = IB_MAD_SIZE;
	for (i = 0; i < N; ++i) {
		int rc;
		if (rc = umad_send(portid, mad_agent, smp_query_tasks[i].umad, length, ibd_timeout, -1) < 0) {
			IBPANIC("send failed rc : %d", rc);
			exit (-1);
		}
		gettimeofday(&smp_query_tasks[i].t1, NULL);
		total_mads++;
		totalSend += length;
	}

	while(!do_poll) {
		do_poll = umad_poll(portid, 1);

		if (do_poll && do_poll != -ETIMEDOUT)
			IBPANIC("poll failed");
			
		if (do_poll == -ETIMEDOUT)
			timeout_mads++;
	
		be64_t tid;

		if (umad_recv(portid, umad, &length, -1) != mad_agent) {
			IBPANIC("recv error: %s", strerror(errno));
			exit (-1);
		}

		gettimeofday(&t2, NULL);
		totalRecv += length;
	
		if (smp->status) {
			fprintf(stdout, "SMP status: 0x%x\n",
				ntohs(smp->status));
			goto exit;
		}

		tid = smp->tid >> 32;

		for (j = 0; j < N; ++j) {
			be64_t t = smp_query_tasks[j].smp->tid >> 32;
			if(t == tid)
				break;
		}
		if(j < N) {
			int rc;

			elapsedTime = (t2.tv_sec - smp_query_tasks[j].t1.tv_sec) * 1000000 + (t2.tv_usec - smp_query_tasks[j].t1.tv_usec);
			if (!minTime || minTime > elapsedTime)
				minTime = elapsedTime;
			if (maxTime < elapsedTime)
				maxTime = elapsedTime;
			totalTime += elapsedTime;
			avrgTime = totalTime / total_mads;

			if (rc = umad_send(portid, mad_agent, smp_query_tasks[j].umad, length, ibd_timeout, ibd_retries) < 0) {
				IBPANIC("send failed rc : %d", rc);
				exit (-1);
			}
			total_mads++;
			totalSend += length;
			gettimeofday(&smp_query_tasks[j].t1, NULL);
		} else {
			IBPANIC("can't find tid %lld", tid);
		}
		
		gettimeofday(&end, NULL);
		runTime = (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec) / 1000000;
		if (runTime > t_sec)
			do_poll = 1;
	}
	
	printf("# of MADs %ld , # of timeouts %ld\n", total_mads, timeout_mads);
	printf("Min latency %ld us , Max latency %ld us, Average latency %ld us\n", minTime, maxTime, totalTime / total_mads);
    printf("Sent bytes %ld , Recv bytes %ld\n", totalSend, totalRecv);
	printf("Send BW [MB/s] %f\n", totalSend/1024/1024/runTime);
	printf("Send MADS/s %d\n", (int) (total_mads / runTime));
	//printf("Recv BW [MB/s] %d", 1000000*totalRecv/1024/1024/totalTime);

	if (ibdebug)
		fprintf(stderr, "%d bytes received\n", length);

	if (!dump_char) {
		xdump(stdout, NULL, smp->data, 64);
		if (smp->status)
			fprintf(stdout, "SMP status: 0x%x\n",
				ntohs(smp->status));
		goto exit;
	}

	desc = smp->data;
	for (i = 0; i < 64; ++i) {
		if (!desc[i])
			break;
		putchar(desc[i]);
	}
	putchar('\n');
	if (smp->status)
		fprintf(stdout, "SMP status: 0x%x\n", ntohs(smp->status));

exit:
	umad_free(umad);
	for (i = 0; i < N; ++i)
		umad_free(smp_query_tasks[i].umad);
	free(smp_query_tasks);
	return 0;
}
