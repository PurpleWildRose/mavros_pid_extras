/**
 * @brief MAVConn class interface
 * @file interface.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavconn
 * @{
 */
/*
 * libmavconn
 * Copyright 2013,2014,2015,2016 Vladimir Ermakov, All rights reserved.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <set>
#include <cassert>

#include <mavros_proxy/console_bridge_compat.h>
#include <mavros_proxy/interface.h>
#include <mavros_proxy/msgbuffer.h>
#include <mavros_proxy/serial.h>
#include <mavros_proxy/udp.h>
#include <mavros_proxy/tcp.h>

namespace mavconn {
#define PFX	"mavconn: "

using mavlink::mavlink_message_t;
using mavlink::mavlink_status_t;

// static members
std::once_flag MAVConnInterface::init_flag;
std::unordered_map<mavlink::msgid_t, const mavlink::mavlink_msg_entry_t*> MAVConnInterface::message_entries {};
std::atomic<size_t> MAVConnInterface::conn_id_counter {0};


MAVConnInterface::MAVConnInterface(uint8_t system_id, uint8_t component_id) :
	sys_id(system_id),
	comp_id(component_id),
	m_parse_status {},
	m_buffer {},
	m_mavlink_status {},
	tx_total_bytes(0),
	rx_total_bytes(0),
	last_tx_total_bytes(0),
	last_rx_total_bytes(0),
	last_iostat(steady_clock::now())
{
	// 将 conn_id_counter 的值原子地加 1（自增操作不被其他线程中断）
	conn_id = conn_id_counter.fetch_add(1);
	// 确保某个函数仅被执行一次，即使在多线程环境下被多次调用也能保证这一点。
	std::call_once(init_flag, init_msg_entry);
}

mavlink_status_t MAVConnInterface::get_status()
{
	return m_mavlink_status;
}

MAVConnInterface::IOStat MAVConnInterface::get_iostat()
{
	std::lock_guard<std::recursive_mutex> lock(iostat_mutex);
	IOStat stat;

	stat.tx_total_bytes = tx_total_bytes;
	stat.rx_total_bytes = rx_total_bytes;

	auto d_tx = stat.tx_total_bytes - last_tx_total_bytes;
	auto d_rx = stat.rx_total_bytes - last_rx_total_bytes;
	last_tx_total_bytes = stat.tx_total_bytes;
	last_rx_total_bytes = stat.rx_total_bytes;

	auto now = steady_clock::now();
	auto dt = now - last_iostat;
	last_iostat = now;

	float dt_s = std::chrono::duration_cast<std::chrono::seconds>(dt).count();

	stat.tx_speed = d_tx / dt_s;
	stat.rx_speed = d_rx / dt_s;

	return stat;
}

void MAVConnInterface::iostat_tx_add(size_t bytes)
{
	tx_total_bytes += bytes;
}

void MAVConnInterface::iostat_rx_add(size_t bytes)
{
	rx_total_bytes += bytes;
}

void MAVConnInterface::parse_buffer(const char *pfx, uint8_t *buf, const size_t bufsize, size_t bytes_received)
{
	mavlink::mavlink_message_t message;

	assert(bufsize >= bytes_received);

	iostat_rx_add(bytes_received);
	for (; bytes_received > 0; bytes_received--) {
		auto c = *buf++;

		// 逐字节解析输入的二进制数据流，从中提取完整的 MAVLink 消息帧。它负责处理消息的帧同步、字段解析、校验等底层工作，是字节流到结构化消息的关键转换函数。
		auto msg_received = static_cast<Framing>(mavlink::mavlink_frame_char_buffer(&m_buffer, &m_parse_status, c, &message, &m_mavlink_status));

		if (msg_received != Framing::incomplete) {
			log_recv(pfx, message, msg_received);

			if (message_received_cb)
				message_received_cb(&message, msg_received);
		}
	}
}

void MAVConnInterface::log_recv(const char *pfx, mavlink_message_t &msg, Framing framing)
{
	const char *framing_str = (framing == Framing::ok) ? "OK" :
			(framing == Framing::bad_crc) ? "!CRC" :
			(framing == Framing::bad_signature) ? "!SIG" : "ERR";

	const char *proto_version_str = (msg.magic == MAVLINK_STX) ? "v2.0" : "v1.0";

	CONSOLE_BRIDGE_logDebug("%s%zu: recv: %s %4s Message-Id: %u [%u bytes] IDs: %u.%u Seq: %u",
			pfx, conn_id,
			proto_version_str,
			framing_str,
			msg.msgid, msg.len, msg.sysid, msg.compid, msg.seq);
}

void MAVConnInterface::log_send(const char *pfx, const mavlink_message_t *msg)
{
	const char *proto_version_str = (msg->magic == MAVLINK_STX) ? "v2.0" : "v1.0";

	CONSOLE_BRIDGE_logDebug("%s%zu: send: %s Message-Id: %u [%u bytes] IDs: %u.%u Seq: %u",
			pfx, conn_id,
			proto_version_str,
			msg->msgid, msg->len, msg->sysid, msg->compid, msg->seq);
}

// PATH: /opt/ros/noetic/mavlink/v2/message.h
void MAVConnInterface::log_send_obj(const char *pfx, const mavlink::Message &msg)
{
	CONSOLE_BRIDGE_logDebug("%s%zu: send: %s", pfx, conn_id, msg.to_yaml().c_str());
}

void MAVConnInterface::send_message_ignore_drop(const mavlink::mavlink_message_t *msg)
{
	try {
		send_message(msg);
	}
	catch (std::length_error &e) {
		CONSOLE_BRIDGE_logError(PFX "%zu: DROPPED Message-Id %u [%u bytes] IDs: %u.%u Seq: %u: %s",
				conn_id,
				msg->msgid, msg->len, msg->sysid, msg->compid, msg->seq,
				e.what());
	}
}

void MAVConnInterface::send_message_ignore_drop(const mavlink::Message &msg, uint8_t source_compid)
{
	try {
		send_message(msg, source_compid);
	}
	catch (std::length_error &e) {
		CONSOLE_BRIDGE_logError(PFX "%zu: DROPPED Message %s: %s",
				conn_id,
				msg.get_name().c_str(),
				e.what());
	}
}

void MAVConnInterface::set_protocol_version(Protocol pver)
{
	if (pver == Protocol::V10) {
		m_mavlink_status.flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
		m_parse_status.flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
	
	}
	else {
		m_mavlink_status.flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
		m_parse_status.flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
	}
}

Protocol MAVConnInterface::get_protocol_version()
{
	if (m_mavlink_status.flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1)
		return Protocol::V10;
	else
		return Protocol::V20;
}

/**
 * Parse host:port pairs
 * 		主要功能是从输入的字符串中提取主机名（或 IP 地址）和端口号，并在解析失败或格式不完整时使用默认值。
 */
static void url_parse_host(std::string host,
		std::string &host_out, int &port_out,
		const std::string def_host, const int def_port)
{
	std::string port;

	auto sep_it = std::find(host.begin(), host.end(), ':');
	if (sep_it == host.end()) {
		// host
		if (!host.empty()) {
			host_out = host;
			port_out = def_port;
		}
		else {
			host_out = def_host;
			port_out = def_port;
		}
		return;
	}

	if (sep_it == host.begin()) {
		// :port
		host_out = def_host;
	}
	else {
		// host:port
		host_out.assign(host.begin(), sep_it);
	}

	port.assign(sep_it + 1, host.end());
	port_out = std::stoi(port);
}

/**
 * Parse ?ids=sid,cid
 * 		解析 URL 查询参数的工具函数，主要功能是从查询字符串中提取 sysid（系统 ID）和 compid（组件 ID），这两个参数在 MAVLink 协议中用于标识消息的发送方或接收方。
 */
static void url_parse_query(std::string query, uint8_t &sysid, uint8_t &compid)
{
	const std::string ids_end("ids=");
	std::string sys, comp;

	if (query.empty())
		return;

	auto ids_it = std::search(query.begin(), query.end(),
			ids_end.begin(), ids_end.end());
	if (ids_it == query.end()) {
		CONSOLE_BRIDGE_logWarn(PFX "URL: unknown query arguments");
		return;
	}

	std::advance(ids_it, ids_end.length());
	auto comma_it = std::find(ids_it, query.end(), ',');
	if (comma_it == query.end()) {
		CONSOLE_BRIDGE_logError(PFX "URL: no comma in ids= query");
		return;
	}

	sys.assign(ids_it, comma_it);
	comp.assign(comma_it + 1, query.end());

	sysid = std::stoi(sys);
	compid = std::stoi(comp);

	CONSOLE_BRIDGE_logDebug(PFX "URL: found system/component id = [%u, %u]", sysid, compid);
}

static MAVConnInterface::Ptr url_parse_serial(
		std::string path, std::string query,
		uint8_t system_id, uint8_t component_id, bool hwflow)
{
	std::string file_path;
	int baudrate;

	// /dev/ttyACM0:57600
	url_parse_host(path, file_path, baudrate, MAVConnSerial::DEFAULT_DEVICE, MAVConnSerial::DEFAULT_BAUDRATE);
	url_parse_query(query, system_id, component_id);

	// 创建一个 MAVConnSerial 类的实例，并将其封装为 std::shared_ptr<MAVConnInterface> 智能指针返回
	return std::make_shared<MAVConnSerial>(system_id, component_id,
			file_path, baudrate, hwflow);
}

static MAVConnInterface::Ptr url_parse_udp(
		std::string hosts, std::string query,
		uint8_t system_id, uint8_t component_id, bool is_udpb, bool permanent_broadcast)
{
	std::string bind_pair, remote_pair;
	std::string bind_host, remote_host;
	int bind_port, remote_port;

	auto sep_it = std::find(hosts.begin(), hosts.end(), '@');
	if (sep_it == hosts.end()) {
		CONSOLE_BRIDGE_logError(PFX "UDP URL should contain @!");
		throw DeviceError("url", "UDP separator not found");
	}

	bind_pair.assign(hosts.begin(), sep_it);
	remote_pair.assign(sep_it + 1, hosts.end());

	// udp://0.0.0.0:14555@ip:14550
	url_parse_host(bind_pair, bind_host, bind_port, "0.0.0.0", MAVConnUDP::DEFAULT_BIND_PORT);
	url_parse_host(remote_pair, remote_host, remote_port, MAVConnUDP::DEFAULT_REMOTE_HOST, MAVConnUDP::DEFAULT_REMOTE_PORT);
	url_parse_query(query, system_id, component_id);

	if (is_udpb)
		remote_host = permanent_broadcast ? MAVConnUDP::PERMANENT_BROADCAST_REMOTE_HOST : MAVConnUDP::BROADCAST_REMOTE_HOST;

	return std::make_shared<MAVConnUDP>(system_id, component_id,
			bind_host, bind_port,
			remote_host, remote_port);
}

static MAVConnInterface::Ptr url_parse_tcp_client(
		std::string host, std::string query,
		uint8_t system_id, uint8_t component_id)
{
	std::string server_host;
	int server_port;

	// tcp://localhost:5760
	url_parse_host(host, server_host, server_port, "localhost", 5760);
	url_parse_query(query, system_id, component_id);

	return std::make_shared<MAVConnTCPClient>(system_id, component_id,
			server_host, server_port);
}

static MAVConnInterface::Ptr url_parse_tcp_server(
		std::string host, std::string query,
		uint8_t system_id, uint8_t component_id)
{
	std::string bind_host;
	int bind_port;

	// tcp-l://0.0.0.0:5760
	url_parse_host(host, bind_host, bind_port, "0.0.0.0", 5760);
	url_parse_query(query, system_id, component_id);

	return std::make_shared<MAVConnTCPServer>(system_id, component_id,
			bind_host, bind_port);
}

MAVConnInterface::Ptr MAVConnInterface::open_url_no_connect(
		std::string url,
		uint8_t system_id,
		uint8_t component_id)
{
	/* Based on code found here:
	 * http://stackoverflow.com/questions/2616011/easy-way-to-parse-a-url-in-c-cross-platform
	 */

	const std::string proto_end("://");
	std::string proto;
	std::string host;
	std::string path;
	std::string query;

	auto proto_it = std::search(
			url.begin(), url.end(),
			proto_end.begin(), proto_end.end());
	if (proto_it == url.end()) {
		// looks like file path
		CONSOLE_BRIDGE_logDebug(PFX "URL: %s: looks like file path", url.c_str());
		return url_parse_serial(url, "", system_id, component_id, false);
	}

	// copy protocol
	proto.reserve(std::distance(url.begin(), proto_it));
	std::transform(url.begin(), proto_it,
			std::back_inserter(proto),
			std::ref(tolower));

	// copy host
	std::advance(proto_it, proto_end.length());
	auto path_it = std::find(proto_it, url.end(), '/');
	std::transform(proto_it, path_it,
			std::back_inserter(host),
			std::ref(tolower));

	// copy path, and query if exists
	auto query_it = std::find(path_it, url.end(), '?');
	path.assign(path_it, query_it);
	if (query_it != url.end())
		++query_it;
	query.assign(query_it, url.end());

	CONSOLE_BRIDGE_logDebug(PFX "URL: %s: proto: %s, host: %s, path: %s, query: %s",
			url.c_str(), proto.c_str(), host.c_str(),
			path.c_str(), query.c_str());

	MAVConnInterface::Ptr interface_ptr;

	if (proto == "udp")
		interface_ptr = url_parse_udp(host, query, system_id, component_id, false, false);
	else if (proto == "udp-b")
		interface_ptr = url_parse_udp(host, query, system_id, component_id, true, false);
	else if (proto == "udp-pb")
		interface_ptr = url_parse_udp(host, query, system_id, component_id, true, true);
	else if (proto == "tcp")
		interface_ptr = url_parse_tcp_client(host, query, system_id, component_id);
	else if (proto == "tcp-l")
		interface_ptr = url_parse_tcp_server(host, query, system_id, component_id);
	else if (proto == "serial")
		interface_ptr = url_parse_serial(path, query, system_id, component_id, false);
	else if (proto == "serial-hwfc")
		interface_ptr = url_parse_serial(path, query, system_id, component_id, true);
	else
		throw DeviceError("url", "Unknown URL type");

	return interface_ptr;
}

MAVConnInterface::Ptr MAVConnInterface::open_url(
		std::string url,
		uint8_t system_id,
		uint8_t component_id,
		const ReceivedCb &cb_handle_message,
		const ClosedCb &cb_handle_closed_port)
{
	auto interface_ptr = open_url_no_connect(url, system_id, component_id);
	if (interface_ptr)
	{
		if (!cb_handle_message)
		{
			CONSOLE_BRIDGE_logWarn(PFX "You did not provide message handling callback to open_url(), it is unsafe to set it later.");
		}
		interface_ptr->connect(cb_handle_message, cb_handle_closed_port);
	}

	return interface_ptr;

}

}	// namespace mavconn
